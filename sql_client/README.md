# SQL 클라이언트 연동

이 디렉터리는 IoT 서버와 통신하면서 MySQL 데이터베이스를 갱신하고,
필요 시 다른 클라이언트(QT 등)로 메시지를 전달하는 콘솔 프로그램을 제공합니다.

## 기능 요약

- IoT 서버에서 수신한 `STATE`, `TIME` 메시지를 파싱해 `vehicle_zone` 테이블을 업데이트.
- 업데이트된 신호등 정보를 `[JAB_QT]...` 형태로 재전송하여 GUI/다른 클라이언트에 브로드캐스트.
- MySQL 연결/쿼리 실패 시 상세 에러 메시지를 출력해 문제 해결을 돕습니다.

## 구성 파일

- `iot_client_light.c` : IoT 서버에 접속해 신호등 상태를 수신, DB 업데이트를 수행하는 클라이언트
- `Makefile` : 빌드 스크립트 (`gcc` 사용, `-lmysqlclient` 링크)
- `db.txt` : 예제용 DB 정보/쿼리 스니펫
- `temp/` : 임시 파일이나 로그를 저장하는 용도(비어 있을 수 있음)

## 빌드

MySQL 클라이언트 라이브러리(`libmysqlclient`)와 헤더가 설치되어 있어야 합니다.
우분투 기준:

```bash
sudo apt-get install libmysqlclient-dev
make
```

빌드 결과물은 `iot_client_light` 실행 파일로 생성됩니다.

## 실행

```bash
./iot_client_light <server_ip> <port> <id>
```

예: `./iot_client_light 127.0.0.1 9000 JAB_SQL`

프로그램이 시작되면

1. `[ID:PASSWD]` 형식으로 IoT 서버에 로그인합니다.
2. 서버로부터 수신한 메시지를 파싱해 MySQL DB(`v2x_platform` 스키마)를 갱신합니다.
   - `STATE` 메시지 → `vehicle_zone.state_led` 업데이트
   - `TIME` 메시지 → `vehicle_zone.time_led` 및 `state_led` 업데이트 후 QT로 브로드캐스트
3. `[JAB_QT]...` 메시지를 작성해 서버로 다시 전송, GUI 클라이언트가 즉시 반영할 수 있게 합니다.

DB 접속 정보는 `iot_client_light.c` 내부 상수(`host`, `user`, `pass`, `dbname`)에서 수정할 수 있습니다.

## 정리

```bash
make clean
```

을 실행하면 오브젝트/실행 파일이 제거됩니다.

