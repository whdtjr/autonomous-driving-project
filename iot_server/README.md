# IoT TCP 서버/클라이언트

이 디렉터리는 TCP 기반 인증 서버(`iot_server`)와 콘솔 클라이언트(`iot_client`) 예제를
포함합니다. 각 노드는 `idpasswd.txt`에 등록된 ID/PW를 사용해 로그인하며,
`[목적ID]메시지` 포맷으로 메시지를 중계합니다.

## 기능 요약

- `idpasswd.txt` 기반 사용자 인증 및 동시 접속 관리(MAX_CLNT 32개).
- `[대상ID]본문` 포맷을 해석해 지정 ID 혹은 전체(`ALLMSG`)로 메시지 라우팅.
- `IDLIST`, `GETTIME` 등 특별 명령을 처리해 현재 접속자 목록, 서버 시간 반환.
- 중복 로그인 감지, 로그 출력, 뮤텍스로 안전한 클라이언트 카운트 관리.

## 빌드

```bash
make
```

생성물:

- `iot_server` : 서버 실행 파일
- `iot_client` : 터미널 기반 테스트 클라이언트

정리하려면 `make clean`을 실행합니다.

## 로그온 계정 관리

`idpasswd.txt` 파일에 `ID PW` 형식으로 한 줄씩 추가하면 됩니다. 서버는 실행 시 이 파일을
읽어 허용된 계정 목록을 메모리에 로딩합니다.

## 서버 실행

```bash
./iot_server <port>
```

예: `./iot_server 9000`

서버는 다중 클라이언트를 다룰 수 있으며, 메시지는 `[대상ID]본문` 형태로 전달됩니다.
특수 키워드:

- `ALLMSG` : 모든 접속자에게 브로드캐스트
- `IDLIST` : 현재 접속 중인 ID 목록 요청
- `GETTIME` : 서버 시각 응답

## 클라이언트 실행

```bash
./iot_client <server_ip> <port> <id>
```

예: `./iot_client 127.0.0.1 9000 STM`

프로그램이 시작되면 자동으로 `[ID:PASSWD]` 로그인 메시지를 보내고, 이후 콘솔에서
`[대상ID]본문` 또는 일반 문자열(자동으로 `[ALLMSG]`로 래핑)을 입력하면 서버로 전송됩니다.

## ROS 2 연동 예시

`ros2_client` 패키지와 함께 사용하면 STM 등에서 전달한
`[ROS2]RED/GREEN/YELLOW` 메시지를 ROS 2 토픽으로 중계하여 차량 제어에 활용할 수 있습니다.

