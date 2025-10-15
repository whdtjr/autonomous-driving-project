# ROS 2 IoT Traffic Light Client

This package uses a TCP client to connect to the existing IoT server and publishes
incoming traffic light color messages onto a ROS 2 topic.

## Build

```bash
colcon build --packages-select ros2_client
source install/setup.bash
```

## Run

Use the launch file, overriding parameters with environment variables if needed:

```bash
export IOT_SERVER_IP=192.168.0.10
export IOT_SERVER_PORT=9000
export IOT_CLIENT_ID=ROS2
export IOT_TRAFFIC_TOPIC=/traffic_light/color
ros2 launch ros2_client traffic_light_client.launch.py
```

### Parameters

| Name                 | Default Value        | Description                                 |
| -------------------- | -------------------- | ------------------------------------------- |
| `server_ip`          | `127.0.0.1`          | IoT server IP address                       |
| `server_port`        | `9000`               | IoT server TCP port                         |
| `client_id`          | `ROS2`               | ID used for authentication/routing          |
| `password`           | `PASSWD`             | Password used during login handshake        |
| `traffic_topic`      | `/traffic_light/color` | ROS 2 topic for published color messages  |
| `cmd_vel_topic`      | `/cmd_vel`             | Twist 명령을 퍼블리시할 토픽               |
| `reconnect_delay_sec`| `5.0`                | Delay before attempting to reconnect        |
| `green_linear_speed` | `0.5`                | GREEN 수신 시 선속도(m/s)                  |
| `green_angular_speed`| `0.0`                | GREEN 수신 시 각속도(rad/s)                |
| `yellow_linear_speed`| `0.2`                | YELLOW 수신 시 선속도(m/s)                 |
| `yellow_angular_speed`| `0.0`               | YELLOW 수신 시 각속도(rad/s)               |

Messages arriving in the format `[SENDER]PAYLOAD` are published as `std_msgs/msg/String`
with `PAYLOAD` assigned to `data`.  
동시에 `PAYLOAD`가 `RED`, `GREEN`, `YELLOW`일 경우 각각 정지/주행/감속 속도가
`geometry_msgs/msg/Twist`로 `cmd_vel_topic`에 전달됩니다(RED는 0, 나머지는 위
파라미터 값 사용).
