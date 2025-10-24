# Unified Qt – Traffic Light + ONVIF RTSP Viewer

## 📘 프로젝트 개요
이 프로젝트는 **Qt Widgets 기반**의 애플리케이션으로,  
기존 **교통신호등 UI**에 **ONVIF 카메라 탐색 및 RTSP 영상 재생 기능**을 통합한 시스템이다.

- **좌측 패널**: ONVIF 카메라 탐색 및 RTSP 실시간 스트리밍  
- **우측 패널**: 신호등(빨간불·노란불·초록불) 제어, 버튼, 타이머, 구역 표시 등  
- **목표**: 차량 및 보행자 상황에 맞춘 실시간 영상 표시와 신호 제어 기능 통합

---

## 🧩 주요 구성

| 폴더/파일 | 설명 |
|------------|------|
| `src/` | 핵심 소스 코드 및 UI 파일 |
| ├─ `main.cpp` | 프로그램 진입점 |
| ├─ `mainwidget.cpp/h/ui` | 전체 메인 위젯 (신호등 + 카메라 뷰어 UI) |
| ├─ `onvif_browser_widget.cpp/h` | ONVIF 카메라 탐색 및 RTSP 재생 기능 |
| ├─ `socketclient.cpp/h` | TCP/IP 클라이언트 (서버 메시지 송수신) |
| └─ `images.qrc` | 리소스 정의 (res/images 포함) |
| `res/images/` | 신호등 상태 이미지 (green/red/yellow) |
| `unified_qt.pro` | qmake 빌드 설정 파일 |
| `README.md` | 프로젝트 설명서 |

---

## 🛠️ 빌드 환경

### 1. 의존성
- **Qt 5.x 또는 Qt 6.x** (모듈: `Widgets`, `Network`, `Concurrent`)
- **OpenCV** (FFmpeg 지원 필수, `pkg-config: opencv4`)
- **tinyxml2** (`pkg-config: tinyxml2` 또는 `libtinyxml2-dev`)

### 2. 설치 예시 (Ubuntu 22.04)
```bash
sudo apt update
sudo apt install -y qtbase5-dev libopencv-dev libtinyxml2-dev pkg-config g++
```

---

## ⚙️ 빌드 방법 (qmake)

### Qt5 환경
```bash
qmake
make
```

### Qt6 환경
```bash
qmake6
make
```

---

## ▶️ 실행
```bash
./unified_qt
```

실행 시 좌측에 **ONVIF 카메라 탐색 및 영상 뷰어**,  
우측에 **신호등 제어 UI**가 표시된다.

---
