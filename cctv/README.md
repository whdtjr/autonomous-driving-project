# Camera Inference + ROS2 Subscriber

IP 카메라 RTSP 스트림을 실시간으로 받아 TensorRT 엔진으로 추론하고, 결과를 화면에 시각화하며 일정 주기마다 ROS 2 토픽으로 JSON을 퍼블리시합니다. 구독 노드는 이 JSON을 받아 간단히 파싱/로그합니다.

## 구성 파일
- `camera/camera.py`: 실시간 추론 + 시각화 + ROS 2 퍼블리시(옵션)
- `ai_model/metadata.yaml`: 클래스 이름 정의(예: car_accident, collapse)
- `src/intel7_cam_sub/intel7_cam_sub/subscriber_node.py`: ROS 2 구독 노드 구현

## 동작 흐름(camera.py)
1) 입력 준비
- RTSP 연결(`RTSP_URL`)을 `cv2.VideoCapture`로 열고 저지연 옵션 설정
- 가장 최신 프레임만 유지하는 `LatestFrameStream` 스레드로 프레임 버퍼 최소화

2) 엔진 로드/바인딩
- TensorRT 엔진(`ai_model/best.engine`) 역직렬화 후 컨텍스트 생성
- 모든 텐서에 대해 CUDA 메모리 할당, 바인딩 주소 설정

3) 전처리
- `letterbox`로 패딩 리사이즈(640x640)
- `to_chw_rgb01`로 `CHW`/`RGB`/`[0,1]` 텐서 변환

4) 추론 실행
- 입력을 GPU로 복사 → `execute_v2` 실행 → 첫 번째 출력 텐서 수신

5) 후처리(강화)
- 출력 텐서를 `(N,C)`로 자동 디코드하여 좌표/클래스 확률 계산
- 신뢰도 컷(`CONF_TH`), 최소 크기, 사전 Top-K(`PRE_TOPK`)
- 격자 기반 중복 억제(`GRID_CELL`), DIoU-NMS(`NMS_IOU`), 중심거리 억제(`DIoU_DIST_TH`)
- 클래스별 Top-K(`CLASS_TOPK`), 점수 정렬 후 최종 후보 산출
- 클래스 이름은 `ai_model/metadata.yaml`에서 로드

6) 프레임 간 지속성(간단 트래커)
- `SimplePersistence`: IoU/중심거리 기반 그리디 매칭으로 트랙 유지
- 연속 히트 수(`PERSIST_MIN_HITS`)를 만족할 때만 표시, 누락 허용(`PERSIST_MAX_AGE`)

7) 시각화/출력
- 바운딩박스/레이블/점수와 FPS 오버레이 후 창으로 표시(ESC 종료)
- ROS 2 사용 가능 시, 일정 주기(`PUBLISH_INTERVAL_SEC`, 기본 5초)마다 JSON 퍼블리시

## ROS 2 퍼블리시(카메라 퍼블리셔)
- 노드: `camera_pub`
- 토픽: `detections_json`
- 타입: `std_msgs/msg/String`(payload는 JSON 문자열)
- 페이로드 예시:

```json
{
  "w": 1920,
  "h": 1080,
  "dets": [
    { "xyxy": [x1, y1, x2, y2], "score": 0.87, "cls": 1, "name": "collapse" },
    { "xyxy": [x1, y1, x2, y2], "score": 0.76, "cls": 0, "name": "car_accident" }
  ]
}
```

- 각 항목 설명
  - `w`, `h`: 원본 프레임 크기
  - `dets[i].xyxy`: 좌상단(x1,y1), 우하단(x2,y2) 픽셀 좌표
  - `dets[i].score`: 신뢰도(0~1)
  - `dets[i].cls`: 정수 클래스 ID
  - `dets[i].name`: 클래스 이름(`metadata.yaml` 기준)

퍼블리시는 추론된 결과 중 지속성 필터를 통과한 객체만 포함합니다.

## ROS 2 구독 노드(Subscriber)
- 파일: `src/intel7_cam_sub/intel7_cam_sub/subscriber_node.py`
- 노드: `det_subscriber`
- 동작: `detections_json` 구독 → JSON 파싱 → 수신 개수 로그
- 주요 콜백 로직:
  - 메시지 파싱 실패 시 경고 로그 출력
  - 정상 수신 시 `Received {len(dets)} dets` 로그

## 실행 방법
사전 준비
- GPU + CUDA + TensorRT 환경, Python 패키지(OpenCV, PyCUDA, NumPy, PyYAML 등)
- 엔진 파일: `ai_model/best.engine`
- 메타데이터: `ai_model/metadata.yaml`(클래스명)
- RTSP 정보: `camera/camera.py`의 `RTSP_URL` 수정

카메라 퍼블리셔(추론/시각화/퍼블리시)
- ROS 2 미설치: 퍼블리시만 비활성화되고 추론/시각화는 동작
- 실행: `python3 camera/camera.py`

ROS 2 구독 노드
1) 워크스페이스에서 빌드/설정
   - `colcon build --packages-select intel7_cam_sub`
   - `source install/setup.bash`
2) 실행
   - `ros2 run intel7_cam_sub det_subscriber`

## 주요 파라미터(요약)
- 탐지/억제: `CONF_TH`, `NMS_IOU`, `PRE_TOPK`, `CLASS_TOPK`, `GRID_CELL`, `DIoU_DIST_TH`, `MIN_WH`, `MAX_DET`
- 지속성: `PERSIST_MIN_HITS`, `PERSIST_MAX_AGE`, `MATCH_IOU_TH`, `MATCH_DIST_TH`
- 퍼블리시: `PUBLISH_INTERVAL_SEC`(기본 5초)

## 트러블슈팅
- RTSP 연결 실패: `RTSP_URL` 확인, 네트워크/인증 점검, 방화벽 확인
- 엔진/드라이버 불일치: TensorRT 버전과 엔진 빌드 환경 일치 필요
- PyCUDA 오류: NVIDIA 드라이버/CUDA 설치 및 권한 확인
- ROS 2 미설치: 퍼블리시 비활성화 로그가 출력되며 추론은 계속 동작

## 참고
- 클래스 정의: `ai_model/metadata.yaml`
- 간단 스트림 확인: `camera/only_camera.py`

## 모델 사양 및 준비
- 입력 전처리
  - 해상도: `640x640` 고정(letterbox 패딩 리사이즈)
  - 색상/배치: BGR → RGB, `CHW`, `float32`, `[0,1]` 정규화
  - 배치 크기: `N=1` 고정(엔진은 정적 입력 크기 권장)
- 엔진 요구사항
  - TensorRT 엔진은 NMS가 포함되지 않은 원시 예측 출력이어야 함(EfficientNMS 미사용)
  - 입력 텐서: `1x3x640x640` 정적(shape 고정), dtype `FP32`(FP16 엔진도 입력은 `FP32` 허용)
  - 출력 텐서: 단일 플랫 예측 텐서(첫 번째 출력 사용)
- 출력 형식(NxC)
  - 좌표: `xywh`(중심 x,y, 폭 w, 높이 h). 값이 `<=2.0` 범위로 나오면 내부에서 `IMG_SIZE`로 픽셀 스케일 복원
  - 레이아웃: `C = 4 + nc` 또는 `C = 5 + nc`
    - `5 + nc`인 경우: `obj * cls_max`로 점수 계산
    - `4 + nc`인 경우: `cls_max`로 점수 계산(obj 없음)
  - 클래스 확률: 2클래스 이상이면 softmax, 그 외 sigmoid
- 클래스/메타데이터
  - `ai_model/metadata.yaml`의 `names`가 클래스 순서/개수와 일치해야 함
  - 예시: `0: car_accident`, `1: collapse`

### 엔진 생성 가이드(예시)
다음은 일반적인 YOLOv8 가중치에서 TensorRT 엔진으로 만드는 예시입니다. 실제 환경/버전에 따라 옵션/입력 이름이 다를 수 있습니다.

1) ONNX 내보내기(예: Ultralytics)
- `yolo export model=ai_model/best.pt format=onnx imgsz=640 dynamic=False simplify=True opset=12`
  - 또는 PyTorch→ONNX로 직접 export 시 입력 크기를 고정하고 NMS를 포함하지 않도록 구성

2) TensorRT 엔진 빌드(trtexec)
- 입력 이름은 ONNX에 따라 다릅니다(대표적으로 `images`). 아래에서 `<INPUT>`을 실제 입력 이름으로 바꿔주세요.
- FP16 예시(권장):
  - `trtexec --onnx=ai_model/best.onnx --saveEngine=ai_model/best.engine \
             --explicitBatch --minShapes=<INPUT>:1x3x640x640 \
             --optShapes=<INPUT>:1x3x640x640 --maxShapes=<INPUT>:1x3x640x640 \
             --fp16 --workspace=4096`
- NMS가 포함된 변환 옵션/플러그인은 사용하지 마세요(EfficientNMS 미포함 필요)

3) 교체 체크리스트
- [ ] 입력 크기/배치: `1x3x640x640` 정적, 배치 1
- [ ] 출력: 원시 예측(단일 텐서), NMS 없음
- [ ] 클래스 수와 `metadata.yaml` 일치
- [ ] 엔진 경로: `camera/camera.py`의 `ENGINE` 경로 업데이트

### 정확도/성능 팁
- 검출 튜닝: `CONF_TH`, `NMS_IOU`, `CLASS_TOPK`, `GRID_CELL`, `MIN_WH`
- 과검출 억제: 격자 억제, DIoU-NMS, 중심거리 억제(매개변수는 코드 상단 상수)
- 일관성: `PERSIST_MIN_HITS`, `PERSIST_MAX_AGE`, `MATCH_IOU_TH`, `MATCH_DIST_TH`
- 속도: FP16 엔진 사용, `PRE_TOPK`로 후보 제한, `CLASS_TOPK` 조정

