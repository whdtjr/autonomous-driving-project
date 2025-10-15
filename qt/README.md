# Qt 신호등 모니터

이 디렉터리는 Qt 위젯 기반의 신호등 모니터 애플리케이션(`traffic_light`)을 포함합니다.
IoT 서버와 TCP로 연결해 수신한 신호등 상태를 GUI에 표시합니다.

## 기능 요약

- IoT 서버 접속 후 `[ID:PASSWD]` 자동 로그인.
- READY 상태에서 수신한 신호등 정보 파싱 및 UI 업데이트.
- 서버 연결 끊김/에러를 팝업으로 안내하고 재접속을 유도.
- 수동 버튼 조작으로 테스트용 신호등 상태 순환 표시.

## 프로젝트 구조

- `traffic_light/traffic_light.pro` : Qt 프로젝트 파일
- `mainwidget.*` : 메인 위젯, 수신 데이터 파싱 및 UI 갱신
- `socketclient.*` : TCP 접속 및 로그인/수신 처리
- `images/` : 신호등 이미지 리소스

## 빌드 방법

1. Qt 5/6 개발 환경이 준비돼 있어야 합니다.
2. Qt Creator에서 `traffic_light/traffic_light.pro`를 열고 빌드하거나,
   터미널에서 qmake/clang을 이용해 빌드합니다.

예시(qmake 사용):

```bash
cd traffic_light
qmake
make
```

빌드 결과물은 `traffic_light` 실행 파일로 생성됩니다.

## 실행

```bash
cd traffic_light
./traffic_light
```

프로그램이 시작되면 서버 IP를 묻는 입력 창이 나타납니다. 기본값은
`socketclient.h`에 정의된 `SERVERIP`(`10.10.16.80`)이며, 필요 시 수정하거나
실행 시 다른 IP를 입력합니다. 연결에 성공하면 자동으로 `[ID:PASSWD]` 로그인
메시지를 전송합니다.

## 동작

- 메시지 형식: `[보낸ID]TIME@시간@색상@존정보`  
  예) `[JAB_STM]TIME@00:10@GREEN@Z1`
- 색상에 따라 이미지와 타이머, 존 정보가 실시간으로 업데이트됩니다.
- `SocketClient` 클래스가 서버와의 연결 유지 및 재시도를 담당합니다.

## 추가 참고

- 클라이언트 ID/비밀번호, 기본 서버 정보는 `socketclient.h`에서 변경할 수 있습니다.
- Qt 리소스(`images.qrc`)에 등록된 이미지 이름을 바꾸면 UI에 표시되는
  신호등 이미지도 함께 변경됩니다.

