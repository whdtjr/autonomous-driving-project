#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, json, threading
from collections import deque

# ===== Optional ROS2 =====
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    _ROS_AVAILABLE = True
except Exception:
    _ROS_AVAILABLE = False

import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ================== 고정 경로/설정 ==================
RTSP_URL   = "rtsp://admin:admin%401234@10.10.16.201:554/0"
MODEL_PATH = "/home/orin/intel7_final_ws/ai_model/train_best.pt"

IMG_SIZE = 640  # Ultralytics 입력 크기
CONF     = 0.60
IOU      = 0.50
MAX_DET  = 30
USE_HALF = True

# 퍼블리시 주기(초)
PUBLISH_INTERVAL_SEC = 5.0

# ================== 프레임 스레드 (LatestFrameStream) ==================
class LatestFrameStream:
    def __init__(self, cap, buffer_size=2, wait_sleep=0.002):
        self.cap = cap
        self.frames = deque(maxlen=max(1, buffer_size))
        self.wait_sleep = float(wait_sleep)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def _reader(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005); continue
            with self.lock:
                self.frames.append(frame)

    def read(self, timeout=0.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if self.frames:
                    frame = self.frames[-1]
                    self.frames.clear()
                    return True, frame
            if not self.running: break
            time.sleep(self.wait_sleep)
        return False, None

# ================== ROS2 퍼블리셔 ==================
class DetPublisher(Node):
    def __init__(self):
        super().__init__('det_publisher')
        self.pub = self.create_publisher(String, 'detections_json', 10)
        self.get_logger().info('Publishing to /detections_json')

    def publish_dets(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        # 로그 간단히
        self.get_logger().info(f"📤 Published {len(payload.get('dets', []))} dets")

# ================== 유틸: 결과를 camera.py 형식으로 직렬화 ==================
def yolo_to_payload(ultra_result, names, W, H) -> dict:
    """
    payload 형식 (camera.py와 호환):
    {
      "w": W, "h": H,
      "dets":[{"xyxy":[x1,y1,x2,y2],"score":0.87,"cls":0,"name":"car_accident"}, ...]
    }
    """
    dets = []
    boxes = getattr(ultra_result, 'boxes', None)
    if boxes is not None and boxes.xyxy is not None and len(boxes) > 0:
        xyxy = boxes.xyxy.detach().cpu().numpy()
        conf = boxes.conf.detach().cpu().numpy()
        cls  = boxes.cls.detach().cpu().numpy().astype(int)
        for i in range(len(xyxy)):
            x1, y1, x2, y2 = [int(v) for v in xyxy[i]]
            cid = int(cls[i])
            name = names[cid] if names and cid in names else str(cid)
            dets.append({
                "xyxy":[x1, y1, x2, y2],
                "score": float(conf[i]),
                "cls": cid,
                "name": name
            })
    return {"w": int(W), "h": int(H), "dets": dets}

def main():
    # ===== ROS2 init =====
    node = None
    if _ROS_AVAILABLE:
        rclpy.init(args=None)
        node = DetPublisher()
    else:
        print("[WARN] ROS2 미탑재: 메시지 퍼블리시는 비활성화됩니다.")

    # ===== 모델 로드 (GPU/FP16) =====
    assert torch.cuda.is_available(), "CUDA not available"
    model = YOLO(MODEL_PATH)
    model.to("cuda")
    try: model.fuse()
    except: pass
    if USE_HALF:
        try:
            model.model.half()
        except Exception:
            try: model.half()
            except: pass
    names = model.names if hasattr(model, "names") else None

    # ===== 저지연 FFmpeg 옵션 세팅 (camera.py 방식) =====
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;tcp|stimeout;5000000|fflags;nobuffer|flags;low_delay|"
        "use_wallclock_as_timestamps;1|max_delay;0|buffer_size;0"
    )
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("❌ IP 카메라 연결 실패")
        if node: node.destroy_node(); 
        if _ROS_AVAILABLE: rclpy.shutdown()
        return
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # ===== 프레임 스레드 시작 =====
    frame_stream = LatestFrameStream(cap)
    frame_stream.start()

    # ===== FPS 계산 =====
    t_prev = 0.0
    fps_hist = deque(maxlen=30)

    # ===== 퍼블리시 간격 =====
    last_pub = 0.0

    print("✅ 시작: GPU .pt 추론 + 저지연 FFmpeg + 프레임 스레드 + 5초 퍼블리시 (/detections_json)")
    try:
        while True:
            ok, frame = frame_stream.read(timeout=0.5)
            if not ok:
                print("⚠️ 프레임 대기 타임아웃")
                # ROS2 콜백 처리
                if node: rclpy.spin_once(node, timeout_sec=0.0)
                continue

            H, W = frame.shape[:2]

            # ===== FPS 갱신 =====
            t_now = time.time()
            dt = t_now - t_prev if t_prev > 0 else 0.0
            fps_inst = (1.0 / dt) if dt > 0 else 0.0
            if fps_inst > 0:
                fps_hist.append(fps_inst)
            fps_avg = sum(fps_hist) / len(fps_hist) if fps_hist else 0.0
            t_prev = t_now

            # ===== 추론 =====
            res = model.predict(source=frame, device=0, half=USE_HALF, verbose=False,
                                imgsz=IMG_SIZE, conf=CONF, iou=IOU, max_det=MAX_DET)

            # ===== 5초마다 ROS2 퍼블리시 =====
            if node is not None:
                now_pub = time.time()
                if now_pub - last_pub >= PUBLISH_INTERVAL_SEC:
                    payload = yolo_to_payload(res[0], names, W, H)
                    node.publish_dets(payload)
                    last_pub = now_pub

            # ===== 시각화 + FPS 오버레이 =====
            show = res[0].plot()
            cv2.putText(show, f"FPS: {fps_avg:.1f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.imshow("YOLO(.pt) Inference (ROS2 + Low-latency FFmpeg)", show)
            if cv2.waitKey(1) & 0xFF == 27:
                break

            # ===== ROS2 스핀 =====
            if node: rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        frame_stream.stop()
        cap.release()
        cv2.destroyAllWindows()
        if node:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
