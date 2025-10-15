# ROS 2 IoT 신호등 클라이언트

이 패키지는 IoT TCP 서버에서 수신한 신호등 색상 메시지를 ROS 2 토픽으로 전달하고,
동시에 로봇의 `cmd_vel`을 제어합니다.

## 기능 정리

- IoT TCP 서버에 자동 연결해 `[ID]` 형태의 신호 메시지를 수신합니다.
- 수신한 색상(RED, GREEN, YELLOW)을 `std_msgs/String` 형태로 `/traffic_light/color` 토픽에 퍼블리시합니다.
- 같은 색상에 맞춰 `geometry_msgs/Twist`를 `/cmd_vel`로 발행해 ROS 이동 속도를 즉시 제어합니다.
  - RED → 정지(0,0)
  - GREEN → 직진 기본 속도
  - YELLOW → 감속 속도
- Launch 파라미터로 서버 IP/포트, 토픽 이름, 색상별 속도를 유연하게 설정할 수 있습니다.
- 연결이 끊어지면 지정한 주기(`reconnect_delay_sec`)로 자동 재접속을 시도합니다.
- 수신 로그와 발행된 속도 정보는 노드 로그(INFO)로 확인할 수 있습니다.

## 빌드

```bash
colcon build --packages-select ros2_client
source install/setup.bash
```

## 실행

```bash
export IOT_SERVER_IP=192.168.0.10  # 필요한 경우 서버 IP를 지정
ros2 launch ros2_client traffic_light_client.launch.py
```

`IOT_SERVER_IP`, `IOT_SERVER_PORT`, `IOT_CLIENT_ID`, `IOT_TRAFFIC_TOPIC` 등의 환경 변수를
설정해 실행 방식을 조정할 수 있습니다.

## 파라미터

| 이름                   | 기본값                 | 설명                                   |
| ---------------------- | ---------------------- | -------------------------------------- |
| `server_ip`            | `127.0.0.1`            | IoT 서버 IP 주소                       |
| `server_port`          | `9000`                 | IoT 서버 TCP 포트                      |
| `client_id`            | `ROS2`                 | 인증·라우팅에 사용할 ID                |
| `password`             | `PASSWD`               | 로그인 시 사용할 비밀번호              |
| `traffic_topic`        | `/traffic_light/color` | 수신 색상을 퍼블리시할 ROS 토픽        |
| `cmd_vel_topic`        | `/cmd_vel`             | Twist 명령을 퍼블리시할 토픽           |
| `reconnect_delay_sec`  | `5.0`                  | 재접속 시도 간격(초)                   |
| `green_linear_speed`   | `0.5`                  | GREEN 수신 시 선속도(m/s)              |
| `green_angular_speed`  | `0.0`                  | GREEN 수신 시 각속도(rad/s)            |
| `yellow_linear_speed`  | `0.2`                  | YELLOW 수신 시 선속도(m/s)             |
| `yellow_angular_speed` | `0.0`                  | YELLOW 수신 시 각속도(rad/s)           |

## 동작

- `[SENDER]PAYLOAD` 형식의 메시지를 수신하면 `PAYLOAD`를 `std_msgs/msg/String`으로
  `traffic_topic`에 퍼블리시합니다.
- `PAYLOAD`가 `RED`, `GREEN`, `YELLOW` 중 하나면 색상에 대응하는 속도를
  `geometry_msgs/msg/Twist`로 `cmd_vel_topic`에 발행합니다.

## 실행

- 워크스페이스 루트에서 빌드

    ```c
    colcon build --packages-select ros2_client
    ```

- 새 터미널을 열 때마다 ROS 2와 워크스페이스 환경 불러오기:

    ```c
    source /opt/ros/humble/setup.bash
    source ~/Workspace/final_project/autonomous-driving-project/install/setup.bash
    ```

- IoT 서버와 통신하는 ROS 2 클라이언트 실행:

    ```c
    ros2 launch ros2_client traffic_light_client.launch.py
    ```

- 속도 명령 확인:

    ```c
    ros2 topic echo /cmd_vel
    ```
