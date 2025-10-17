import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    server_ip = os.getenv("IOT_SERVER_IP", "127.0.0.1")
    server_port = int(os.getenv("IOT_SERVER_PORT", "9000"))
    client_id = os.getenv("IOT_CLIENT_ID", "ROS2")
    traffic_topic = os.getenv("IOT_TRAFFIC_TOPIC", "/traffic_light/color")

    return LaunchDescription(
        [
            Node(
                package="ros2_client",
                executable="traffic_light_client",
                name="traffic_light_tcp_client",
                parameters=[
                    {
                        "server_ip": server_ip,
                        "server_port": server_port,
                        "client_id": client_id,
                        "traffic_topic": traffic_topic,
                    }
                ],
            )
        ]
    )
