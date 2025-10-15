# STM 펌웨어 프로젝트

이 디렉터리는 STM32F411 기반의 STM32CubeIDE 프로젝트들을 담고 있습니다.
신호등 제어 및 센서 테스트 예제가 포함되어 있으며, 각각 별도의 `.ioc` 설정과
`Core/`, `Drivers/` 소스 폴더를 사용합니다.

## 프로젝트 목록

- `traffic_light/` : 신호등(RED, YELLOW, GREEN) LED 제어 및 Wi-Fi 통신 예제  
  - 핀 배치와 간단 설명은 `traffic_light/README.md` 참고
-  - 주요 기능
     - 타이머 기반으로 RED/YELLOW/GREEN LED를 순환 제어
     - 외부 Wi-Fi 모듈을 통해 IoT TCP 서버로 `TIME`, `STATE` 패킷 전송
     - ROS 2 `ros2_client` 노드와 연계해 자율주행 차량 제어 신호로 활용
- `mpu9250_test/` : MPU9250 IMU 센서 테스트용 기본 프로젝트(코드 템플릿 포함)
  - 주요 기능
     - I2C 초기화 및 MPU9250 레지스터 구성 예제
     - 가속도/자이로/자기 센서 데이터 읽기 루프 골격
     - 필터링·센서퓨전 로직을 추가해 확장할 수 있는 베이스 코드

## 개발 환경

- STM32CubeIDE 또는 STM32CubeMX + GCC Toolchain
- 타겟 보드: STM32F411 (디폴트 링커 스크립트 `STM32F411RETX_*.ld`)
- 디버깅/플래싱: ST-LINK V2(V3) 등 JTAG/SWD 디버거

## 열기 및 빌드

1. STM32CubeIDE에서 `File -> Open Projects from File System`을 선택합니다.
2. `STM/traffic_light` 또는 `STM/mpu9250_test` 디렉터리를 선택해 임포트합니다.
3. 프로젝트 설정(보드, 클럭, 핀맵)을 확인한 뒤 `Build`/`Debug`를 실행합니다.

## 펌웨어 동작 개요

- `traffic_light` 프로젝트는 각 LED 핀을 주기적으로 제어하며, Wi-Fi 모듈과의
  TCP 통신을 통해 색상/타이머 정보를 외부 서버에 보내도록 확장할 수 있습니다.
- `mpu9250_test` 프로젝트는 IMU 센서 데이터 처리를 위한 기반 코드로, 멀티채널
  데이터 수집 및 필터링 로직을 추가해 활용할 수 있습니다.

필요에 따라 CubeMX 설정(.ioc)을 수정하거나 HAL/LL 코드를 보강해 프로젝트를 확장하세요.
