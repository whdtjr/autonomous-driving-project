#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from vision_msgs.msg import Detection2DArray
from collections import deque
import subprocess
import can
import threading
import time
import socket
import errno
import os
import select
import json
# ===== 설정 =====
CAN_ID_DIST = 0x102
CAN_ID_MAG  = 0x103

STOP_THRESH_MM    = 100   # 100 mm 이하면 정지
RELEASE_THRESH_MM = 150   # 150 mm 이상이면 해제
DIST_TIMEOUT_S    = 0.5   # 거리 수신 타임아웃

# ===== 신호등 ====
GREEN_LINEAR_SPEED = 0.07
YELLOW_LINEAR_SPEED = 0.03
RED_LINEAR_SPEED = 0.0
TL_TIMEOUT_S = 3.0 

SERVER_IP = "10.10.16.80"
SERVER_PORT = 9000

CLIENT_ID = "ROS2"
CLIENT_PASS = "PASSWD"

DB_ID = "V2I_DB"

STATE_STOP = 'stop'
STATE_DRIVING = 'driving'

ROTATION_ANGULAR_SPEED = -1.0  # rad/s, negative -> 오른쪽 회전
ROTATION_DURATION = 1.6  # s, 약 90도 회전 시간

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')

        # ===== ROS I/O =====
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.callback_follow_lane, 1)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.callback_get_max_vel, 1)
        self.sub_avoid_cmd = self.create_subscription(Twist, '/avoid_control', self.callback_avoid_cmd, 1)
        self.sub_avoid_active = self.create_subscription(Bool, '/avoid_active', self.callback_avoid_active, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)
        self.schoolzone_detection_topic = self.declare_parameter(
            'schoolzone_detection_topic', 'detections'
        ).value
        self.schoolzone_label = self.declare_parameter(
            'schoolzone_label', 'b_sz'
        ).value
        self.schoolzone_threshold = int(
            self.declare_parameter('schoolzone_detection_threshold', 8).value
        )
        self.schoolzone_window_sec = float(
            self.declare_parameter('schoolzone_detection_window_sec', 3.0).value
        )
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            self.schoolzone_detection_topic,
            self.callback_detection,
            10,
        )

        self.sub_detections_json = self.create_subscription(
            String,
            'detections_json',
            self.callback_detections_json,
            10,
        )
        self._detections_json_enabled = False

        # ===== CAN 초기화 =====
        self.IFACE = "can0"
        self.BITRATE = 100000  # 100 kbps
        self.bring_up_can()
        self.bus = can.interface.Bus(channel=self.IFACE, bustype="socketcan")
        self.bus.set_filters([
            {"can_id": CAN_ID_DIST, "can_mask": 0x7FF},
            {"can_id": CAN_ID_MAG,  "can_mask": 0x7FF},
        ])

        # ===== 제어 변수 =====
        self.last_error = 0.0

        # 사용자/기본 속도 상한
        self.DEFAULT_MAX_VEL = 0.1
        self.user_max_vel = self.DEFAULT_MAX_VEL  # /control/max_vel 로 업데이트
        self.MAX_VEL = self.DEFAULT_MAX_VEL       # 실제로 사용되는 유효 상한(센서 반영)

        self.SCHOOLZONE_MAX_VEL = 0.05  # 스쿨존 감속 목표 속도
        self.left_turn_gain = 0.8
        self.right_turn_gain = 1.2
        self.left_turn_limit = 1.5
        self.right_turn_limit = 1.5

        # 회피 모드
        self.avoid_active = False
        self.avoid_twist = Twist()

        # ===== 신호등 상태 ====
        self.traffic_state = None      # 'GREEN' | 'YELLOW' | 'RED' | None(미수신)
        self._tl_last_ts   = 0.0
        self._tl_sock      = None
        self._tl_online    = False
        self._tl_state     = 'idle'
        self._tl_enabled   = False
        self._tl_outgoing  = bytearray()
        self._tl_inbuf     = bytearray()
        self._tl_connect_deadline = 0.0
        self._tl_next_retry = 0.0
        self._tl_request_pending = False
        # ===== 센서 상태 =====
        self.obstacle_close = False      # 거리 ≤ 10cm 이면 True
        self.mag_state = None            # 0 or 1 (None=미수신)
        self.last_dist_mm = None
        self.last_dist_ts = 0.0
        self._mag_last_ts = 0.0

        # 디버그 로그 스로틀(지자기)
        self._mag_log_prev_ts = 0.0
        self._mag_log_period = 0.5  # 0.5초마다 한 번씩만 반복 로그
        self._schoolzone_hits = deque()
        self._schoolzone_active = False
        self._latest_detections = []
        self.vehicle_state = STATE_DRIVING
        self._rotation_active = False
        self._rotation_end_time = 0.0

        # 타임아웃 감시(10Hz)
        self._lock = threading.Lock()
        self.create_timer(0.1, self._watchdog_timer)
        self.create_timer(0.05, self._traffic_light_timer)
        self.create_timer(0.1, self._schoolzone_timer)
        self._rotation_timer = self.create_timer(0.05, self._rotation_tick)

        # CAN 리시버 스레드
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(target=self._can_rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info("ControlLane initialized with CAN listener (no debounce).")

    # ========= 콜백들 =========
    def callback_get_max_vel(self, max_vel_msg: Float64):
        with self._lock:
            self.user_max_vel = float(max_vel_msg.data)
        self._recompute_max_vel()

    def callback_follow_lane(self, desired_center: Float64):
        if self.avoid_active:
            return

        center = desired_center.data
        error = center - 500.0

        Kp = 0.0022
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        with self._lock:
            max_vel_now = self.MAX_VEL
            vehicle_state = self.vehicle_state

        if vehicle_state == STATE_STOP:
            self.pub_cmd_vel.publish(Twist())
            return

        twist = Twist()
        # 선속도: 오차 커질수록 감속, 최종 캡은 0.7 (원 코드 유지)
        twist.linear.x = min(max_vel_now * (max(1 - abs(error) / 500.0, 0.0) ** 2.2), 0.7)
        if angular_z < 0.0:
            angular_z *= self.left_turn_gain
            twist.angular.z = -max(angular_z, -self.left_turn_limit)
        else:
            angular_z *= self.right_turn_gain
            twist.angular.z = -min(angular_z, self.right_turn_limit)
        self.pub_cmd_vel.publish(twist)

    def callback_detections_json(self, msg: String):
        with self._lock:
            enabled = self._detections_json_enabled
        if not enabled:
            return
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse detections_json: {exc}')
            return

        dets = data.get('dets', [])
        self._latest_detections = dets
        self.get_logger().debug(f'detections_json update: {len(dets)} dets')

        if dets and self._is_stop_state():
            self._start_right_rotation()

    def callback_avoid_cmd(self, twist_msg: Twist):
        self.avoid_twist = twist_msg
        if self.avoid_active:
            if self._is_stop_state():
                self.pub_cmd_vel.publish(Twist())
            else:
                self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg: Bool):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def callback_detection(self, msg: Detection2DArray):
        if not msg.detections:
            return

        now = time.monotonic()
        hit_recorded = False
        for detection in msg.detections:
            for hypothesis in detection.results:
                if hypothesis.hypothesis.class_id == self.schoolzone_label:
                    hit_recorded = True
                    break
            if hit_recorded:
                break
        if hit_recorded:
            with self._lock:
                self._schoolzone_hits.append(now)
            self.get_logger().debug('Schoolzone detection recorded for slowdown logic.')

    # ========= 종료 처리 =========
    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        self._stop_event.set()
        try:
            self._stop_traffic_light_comm()
            self._rx_thread.join(timeout=1.0)
            self.bus.shutdown()
        except Exception:
            pass
        self.pub_cmd_vel.publish(Twist())  # zero

    def _start_traffic_light_comm(self):
        with self._lock:
            self._tl_enabled = True
            if self._tl_state == 'idle':
                self._tl_next_retry = 0.0
            immediate = self._tl_state == 'connected'
        if immediate:
            self._prepare_v2i_request()

    def _stop_traffic_light_comm(self):
        with self._lock:
            self._tl_enabled = False
            self._tl_request_pending = False
        need_recompute = self._close_traffic_light(reset_state=True)
        self.get_logger().info('[TL] communication stopped by request')
        if need_recompute:
            self._recompute_max_vel()

    def _prepare_v2i_request(self):
        should_log = False
        with self._lock:
            if (
                self._tl_state == 'connected'
                and self._tl_sock is not None
                and self._tl_request_pending
            ):
                self._tl_outgoing.extend(b"[V2I_DB]\n")
                self._tl_request_pending = False
                should_log = True
        if should_log:
            self.get_logger().info('[TL] queued V2I request [V2I_DB]')

    def _queue_v2i_request(self):
        with self._lock:
            self._tl_request_pending = True
        self._prepare_v2i_request()

    def _close_traffic_light(self, *, schedule_retry=None, reset_state=False):
        sock = None
        need_recompute = False
        with self._lock:
            sock = self._tl_sock
            self._tl_sock = None
            self._tl_online = False
            self._tl_state = 'idle'
            self._tl_connect_deadline = 0.0
            self._tl_outgoing.clear()
            self._tl_inbuf.clear()
            if schedule_retry is not None:
                self._tl_next_retry = schedule_retry
            else:
                self._tl_next_retry = 0.0
            if reset_state and self.traffic_state is not None:
                self.traffic_state = None
                self._tl_last_ts = 0.0
                need_recompute = True
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        return need_recompute

    def _traffic_light_timer(self):
        now = time.time()
        with self._lock:
            enabled = self._tl_enabled
            state = self._tl_state
            sock_present = self._tl_sock is not None
            next_retry = self._tl_next_retry
        if not enabled:
            if state != 'idle' or sock_present:
                need = self._close_traffic_light(reset_state=True)
                if need:
                    self._recompute_max_vel()
            return

        if state == 'idle':
            if now >= next_retry:
                self._begin_traffic_light_connect(now)
            return

        if state == 'connecting':
            self._advance_tl_connecting(now)
            return

        if state == 'connected':
            self._advance_tl_connected(now)

    def _begin_traffic_light_connect(self, now):
        self.get_logger().info(f"[TL] Connecting to {SERVER_IP}:{SERVER_PORT} ...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            err = sock.connect_ex((SERVER_IP, SERVER_PORT))
        except Exception as exc:
            self._handle_tl_failure(f'connect error: {exc}', now, reset_state=False)
            return
        if err not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EAGAIN):
            try:
                sock.close()
            except OSError:
                pass
            self._handle_tl_failure(f"connect failed: {os.strerror(err)}", now, reset_state=False)
            return
        with self._lock:
            self._tl_sock = sock
            self._tl_state = 'connecting'
            self._tl_connect_deadline = now + 5.0
            self._tl_outgoing.clear()
            self._tl_inbuf.clear()
            self._tl_online = False
        if err == 0:
            self._on_tl_connected(sock, now)

    def _advance_tl_connecting(self, now):
        with self._lock:
            sock = self._tl_sock
            deadline = self._tl_connect_deadline
        if sock is None:
            self._handle_tl_failure('socket missing during connect', now, reset_state=False)
            return
        try:
            _, writable, exceptional = select.select([], [sock], [sock], 0)
        except Exception as exc:
            self._handle_tl_failure(f'select error: {exc}', now, reset_state=False)
            return
        if exceptional:
            self._handle_tl_failure('connection error reported by socket', now, reset_state=False)
            return
        if writable:
            err = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if err != 0:
                self._handle_tl_failure(f"connection failed: {os.strerror(err)}", now, reset_state=False)
                return
            self._on_tl_connected(sock, now)
            return
        if deadline and now > deadline:
            self._handle_tl_failure('connection timeout', now, reset_state=False)

    def _on_tl_connected(self, sock, now):
        with self._lock:
            if self._tl_sock is not sock:
                return
            self._tl_state = 'connected'
            self._tl_online = True
            self._tl_connect_deadline = 0.0
            self._tl_outgoing.clear()
            self._tl_inbuf.clear()
            self._tl_outgoing.extend(f"[{CLIENT_ID}:{CLIENT_PASS}]".encode())
        self.get_logger().info('[TL] Connected.')
        self._prepare_v2i_request()

    def _advance_tl_connected(self, now):
        with self._lock:
            sock = self._tl_sock
            outgoing_snapshot = bytes(self._tl_outgoing)
        if sock is None:
            self._handle_tl_failure('socket missing while connected', now, reset_state=True)
            return

        self._prepare_v2i_request()

        with self._lock:
            outgoing_snapshot = bytes(self._tl_outgoing)
        if outgoing_snapshot:
            try:
                sent = sock.send(outgoing_snapshot)
            except BlockingIOError:
                sent = 0
            except OSError as exc:
                self._handle_tl_failure(f'send error: {exc}', now, reset_state=True)
                return
            if sent > 0:
                with self._lock:
                    del self._tl_outgoing[:sent]

        try:
            chunk = sock.recv(1024)
        except BlockingIOError:
            chunk = None
        except OSError as exc:
            self._handle_tl_failure(f'recv error: {exc}', now, reset_state=True)
            return
        if chunk is None:
            return
        if not chunk:
            self._handle_tl_failure('server closed connection', now, reset_state=True)
            return
        with self._lock:
            self._tl_inbuf.extend(chunk)
        for raw_line in self._extract_tl_lines():
            self._process_tl_message(raw_line)

    def _extract_tl_lines(self):
        lines = []
        with self._lock:
            buf = self._tl_inbuf
            while True:
                idx = buf.find(b'\n')
                if idx == -1:
                    break
                lines.append(bytes(buf[:idx]))
                del buf[:idx + 1]
        return lines

    def _process_tl_message(self, raw_bytes):
        raw_msg = raw_bytes.decode('utf-8', 'ignore').strip()
        if not raw_msg:
            return
        msg_upper = raw_msg.upper()
        payload = msg_upper
        if payload.startswith('[') and ']' in payload:
            payload = payload.split(']', 1)[1].strip()
        if payload in ('GREEN', 'YELLOW', 'RED'):
            with self._lock:
                prev = self.traffic_state
                self.traffic_state = payload
                self._tl_last_ts = time.time()
            if prev != payload:
                self.get_logger().info(f"[TL] state={payload}")
                self._recompute_max_vel()
        else:
            self.get_logger().warning(f"[TL] invalid payload: {raw_msg}")

    def _handle_tl_failure(self, message, now, reset_state):
        self.get_logger().warning(f"[TL] {message}. reconnecting in 1s ...")
        need = self._close_traffic_light(schedule_retry=now + 1.0, reset_state=reset_state)
        if need:
            self._recompute_max_vel()

    # ========= CAN RX =========
    def _can_rx_loop(self):
        while not self._stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue

                # debug 
                # self.get_logger().info(f"[CAN] ID=0x{msg.arbitration_id:X} DLC={len(msg.data)} DATA={msg.data.hex()}")

                if msg.arbitration_id == CAN_ID_DIST and len(msg.data) >= 2:
                    dist_mm = self.be16(msg.data)   # 단위: mm
                    now = time.time()
                    recompute = False
                    dist_log_msg = None
                    trigger_rotate = False
                    with self._lock:
                        self.last_dist_mm = dist_mm
                        self.last_dist_ts = now

                        prev = self.obstacle_close
                        if prev:
                            if dist_mm >= RELEASE_THRESH_MM:
                                self.obstacle_close = False
                        else:
                            if dist_mm <= STOP_THRESH_MM:
                                self.obstacle_close = True
                                trigger_rotate = True

                        if self.obstacle_close != prev:
                            dist_log_msg = (
                                f"[DIST] {dist_mm}mm → "
                                f"{'CLOSE(<=100mm), STOP' if self.obstacle_close else 'RELEASE(>=150mm), GO'}"
                            )
                            recompute = True
                        else:
                            trigger_rotate = False
                    if dist_log_msg:
                        self.get_logger().info(dist_log_msg)
                    if trigger_rotate:
                        if self._start_right_rotation():
                            self.get_logger().info('거리 센서 감지 → 약 90도 오른쪽 회전 시작')
                    if recompute:
                        self._recompute_max_vel()

                elif msg.arbitration_id == CAN_ID_MAG and len(msg.data) >= 1:
                    val = int(msg.data[0]) & 0x01  # 0 또는 1
                    now = time.time()
                    recompute = False
                    mag_log_msg = None
                    start_tl = False
                    queue_request = False
                    stop_tl = False
                    schoolzone_reset = False
                    activate_detections_json = False
                    with self._lock:
                        prev_state = self.mag_state
                        self.mag_state = val
                        self._mag_last_ts = now

                        if val == 0 and self._schoolzone_active:
                            self._schoolzone_active = False
                            self._schoolzone_hits.clear()
                            schoolzone_reset = True

                        if prev_state != val:
                            mag_log_msg = (
                                f"[MAG] raw={msg.data.hex()} state={val} (velocity control now uses vision detections)"
                            )
                            recompute = True
                            if val == 0:
                                queue_request = True
                                start_tl = True
                            elif val == 1:
                                stop_tl = True
                                if not self._detections_json_enabled:
                                    self._detections_json_enabled = True
                                    activate_detections_json = True
                        else:
                            if (now - self._mag_log_prev_ts) > self._mag_log_period:
                                self._mag_log_prev_ts = now
                                self.get_logger().info(f"[MAG] raw={msg.data.hex()} state={val} (steady)")
                    if mag_log_msg:
                        self.get_logger().info(mag_log_msg)
                    if activate_detections_json:
                        self.get_logger().info("Magnet sensor=1 → detections_json subscription enabled.")
                    if queue_request:
                        self._queue_v2i_request()
                    if start_tl:
                        self._start_traffic_light_comm()
                    if stop_tl:
                        self._stop_traffic_light_comm()
                        recompute = False
                    if schoolzone_reset:
                        self.get_logger().info("Schoolzone slowdown cleared by magnet sensor (value 0).")
                        self._recompute_max_vel()
                    elif recompute:
                        self._recompute_max_vel()

            except Exception as e:
                self.get_logger().warning(f"CAN RX error: {e}")

    # ========= 속도 상한 재계산 =========
    def _recompute_max_vel(self):
        state_changed = False
        max_changed = False
        new_state = None
        new_max_value = None
        with self._lock:
            if self.obstacle_close:
                computed_max = 0.0
            else:
                computed_max = self.user_max_vel
                if self.traffic_state == "RED":
                    computed_max = min(computed_max, RED_LINEAR_SPEED)
                elif self.traffic_state == "YELLOW":
                    computed_max = min(computed_max, YELLOW_LINEAR_SPEED)
                elif self.traffic_state == "GREEN":
                    computed_max = min(computed_max, GREEN_LINEAR_SPEED)
                if self._schoolzone_active:
                    computed_max = min(computed_max, self.SCHOOLZONE_MAX_VEL)
            computed_state = STATE_STOP if (self.obstacle_close or self.traffic_state == "RED") else STATE_DRIVING
            if abs(computed_max - self.MAX_VEL) > 1e-6:
                self.MAX_VEL = computed_max
                max_changed = True
                new_max_value = computed_max
            prev_state = self.vehicle_state
            if computed_state != prev_state:
                self.vehicle_state = computed_state
                state_changed = True
                new_state = computed_state
        if max_changed:
            self.get_logger().info(f"MAX_VEL updated → {new_max_value:.3f} m/s")
        if state_changed:
            self.get_logger().info(f"Vehicle state updated → {new_state}")
            if new_state == STATE_STOP:
                self.pub_cmd_vel.publish(Twist())

    def _start_right_rotation(self) -> bool:
        with self._lock:
            if self._rotation_active:
                return False
            self._rotation_active = True
            self._rotation_end_time = time.monotonic() + ROTATION_DURATION
        self.get_logger().info("Stop 상태 감지 + detections_json → 약 90도 오른쪽 회전 시작")
        return True

    def _rotation_tick(self):
        twist_msg = None
        finished = False
        with self._lock:
            if not self._rotation_active:
                return
            now = time.monotonic()
            if now >= self._rotation_end_time:
                self._rotation_active = False
                finished = True
            else:
                twist_msg = Twist()
                twist_msg.angular.z = ROTATION_ANGULAR_SPEED
        if twist_msg is not None:
            self.pub_cmd_vel.publish(twist_msg)
        elif finished:
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("오른쪽 회전 완료, stop 상태 유지")

    def _is_stop_state(self) -> bool:
        with self._lock:
            return self.vehicle_state == STATE_STOP


    def _schoolzone_timer(self):
        if self.schoolzone_threshold <= 0:
            return

        now = time.monotonic()
        triggered = False
        with self._lock:
            window = self.schoolzone_window_sec
            while self._schoolzone_hits and (now - self._schoolzone_hits[0]) > window:
                self._schoolzone_hits.popleft()

            if not self._schoolzone_active and len(self._schoolzone_hits) >= self.schoolzone_threshold:
                self._schoolzone_active = True
                triggered = True
                self._schoolzone_hits.clear()

        if triggered:
            self.get_logger().info(
                f"Schoolzone slowdown engaged after {self.schoolzone_threshold} detections within {self.schoolzone_window_sec:.1f}s."
            )
            self._recompute_max_vel()

    # ========= 타임아웃 감시 =========
    def _watchdog_timer(self):
        now = time.time()
        recompute = False
        warn_msgs = []
        with self._lock:
            #거리 타임아웃: 정지 상태였는데 프레임이 끊기면 해제(보수적으로 운용하려면 이 부분 제거 가능)
            if self.last_dist_ts != 0 and (now - self.last_dist_ts) > DIST_TIMEOUT_S:
                if self.obstacle_close:
                    self.obstacle_close = False
                    warn_msgs.append(f"[DIST] timeout {DIST_TIMEOUT_S}s → releasing stop (no frames)")
                    recompute = True

            
            if self._tl_last_ts != 0 and (now - self._tl_last_ts) > TL_TIMEOUT_S:
                if self.traffic_state is not None:
                    self.traffic_state = None
                    warn_msgs.append(f"[TL] timeout {TL_TIMEOUT_S}s → state=None")
                    recompute = True
        for msg in warn_msgs:
            self.get_logger().warning(msg)
        if recompute:
            self._recompute_max_vel()

    # ========= 유틸 =========
    def bring_up_can(self):
        cmds = [
            ["sudo", "ip", "link", "set", self.IFACE, "down"],
            ["sudo", "ip", "link", "set", self.IFACE, "type", "can", "bitrate", str(self.BITRATE)],
            ["sudo", "ip", "link", "set", self.IFACE, "up"],
        ]
        for c in cmds:
            subprocess.run(c, check=False)

    def be16(self, b: bytes) -> int:
        return int.from_bytes(b[:2], "big")


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
