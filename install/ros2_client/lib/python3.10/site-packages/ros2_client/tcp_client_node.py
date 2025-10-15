import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TrafficLightTcpClient(Node):
    """ROS 2 node that bridges IoT TCP traffic light messages into a ROS topic."""

    def __init__(self) -> None:
        super().__init__("traffic_light_tcp_client")

        #self.declare_parameter("server_ip", "127.0.0.1")
        self.declare_parameter("server_ip", "10.10.16.80")
        self.declare_parameter("server_port", 9000)
        self.declare_parameter("client_id", "ROS2")
        self.declare_parameter("password", "PASSWD")
        self.declare_parameter("traffic_topic", "/traffic_light/color")
        self.declare_parameter("reconnect_delay_sec", 5.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("green_linear_speed", 0.5)
        self.declare_parameter("green_angular_speed", 0.0)
        self.declare_parameter("yellow_linear_speed", 0.2)
        self.declare_parameter("yellow_angular_speed", 0.0)

        server_ip = str(self.get_parameter("server_ip").value)
        server_port = int(self.get_parameter("server_port").value)
        client_id = str(self.get_parameter("client_id").value)
        password = str(self.get_parameter("password").value)
        topic_name = str(self.get_parameter("traffic_topic").value)
        reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        green_linear_speed = float(self.get_parameter("green_linear_speed").value)
        green_angular_speed = float(self.get_parameter("green_angular_speed").value)
        yellow_linear_speed = float(self.get_parameter("yellow_linear_speed").value)
        yellow_angular_speed = float(self.get_parameter("yellow_angular_speed").value)

        self._client_id = client_id
        self._password = password
        self._server_endpoint = (server_ip, server_port)
        self._reconnect_delay = max(1.0, reconnect_delay)

        self._socket_lock = threading.Lock()
        self._socket: Optional[socket.socket] = None
        self._stop_event = threading.Event()

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._velocities = {
            "RED": (0.0, 0.0),
            "GREEN": (green_linear_speed, green_angular_speed),
            "YELLOW": (yellow_linear_speed, yellow_angular_speed),
        }

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"traffic_light_tcp_client node started; connecting to {server_ip}:{server_port} as {client_id}"
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        with self._socket_lock:
            if self._socket is not None:
                try:
                    self._socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._socket.close()
                self._socket = None
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._connect_and_listen()
            except Exception as exc:  # broad catch to ensure reconnect loop
                self.get_logger().error(f"TCP client error: {exc}")
            if not self._stop_event.is_set():
                time.sleep(self._reconnect_delay)

    def _connect_and_listen(self) -> None:
        server_ip, server_port = self._server_endpoint
        self.get_logger().info(f"Connecting to IoT server {server_ip}:{server_port}")

        sock = socket.create_connection(self._server_endpoint, timeout=5.0)
        sock.settimeout(1.0)

        with self._socket_lock:
            self._socket = sock

        login_payload = f"[{self._client_id}:{self._password}]"
        sock.sendall(login_payload.encode("utf-8"))

        buffer = ""
        while not self._stop_event.is_set():
            try:
                data = sock.recv(1024)
            except socket.timeout:
                continue
            if not data:
                raise ConnectionError("server closed connection")
            buffer += data.decode("utf-8", errors="ignore")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                self._handle_message(line.strip())

    def _handle_message(self, message: str) -> None:
        if not message:
            return

        # Expected format: [SENDER]payload
        sender = None
        payload = message
        if message.startswith("[") and "]" in message:
            end_idx = message.find("]")
            sender = message[1:end_idx]
            payload = message[end_idx + 1 :].strip()

        log_parts = []
        if sender:
            log_parts.append(f"from {sender}")
        if payload:
            log_parts.append(f"payload '{payload}'")
        detail = ", ".join(log_parts) if log_parts else message
        self.get_logger().info(f"Received {detail}")

        if payload:
            msg = String()
            msg.data = payload
            self.publisher_.publish(msg)
            self._publish_cmd_vel(payload)

    def _publish_cmd_vel(self, payload: str) -> None:
        command = payload.strip().upper()
        linear_x, angular_z = self._velocities.get(
            command, (None, None)
        )
        if linear_x is None or angular_z is None:
            self.get_logger().debug(f"Ignoring payload without velocity mapping: {payload}")
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"Published cmd_vel for {command}: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = TrafficLightTcpClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
