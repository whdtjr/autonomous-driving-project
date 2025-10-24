#!/usr/bin/env python3
import os
import cv2
import time
import json
from ultralytics import YOLO

# ========== ROS2 ==========
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# ====== 1) RTSP IP카메라 주소 입력 ======
rtsp_url = "rtsp://admin:admin%401234@10.10.16.201:554/0"  # 카메라 주소

# ====== 2) 모델 로드 (YOLOv8) ======
model = YOLO("runs/detect/train/weights/best.pt")  # 학습 모델(best.pt)

# ====== 3) 임계값/쿨다운 설정 ======
CONF_TH = 0.5       # 신뢰도 임계값
ALERT_COOLDOWN = 2.0  # 초 단위 (도배 방지)

def main():
    # ---- ROS2 초기화 & 퍼블리셔 준비 ----
    rclpy.init()
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,   # 서브스크라이버 QoS와 일치
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    node = rclpy.create_node("yolo_publisher")
    pub = node.create_publisher(String, "/detections_json", qos)  # ★ 토픽명 변경

    # ---- 카메라 연결 ----
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("❌ IP 카메라 연결 실패")
        node.destroy_node()
        rclpy.shutdown()
        return

    last_pub_ts = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 프레임 수신 실패")
                continue

            # AI 추론
            results = model(frame, conf=CONF_TH, iou=0.45, verbose=False)
            r0 = results[0]

            # ---- dets 리스트 구성 (서브스크라이버 포맷) ----
            dets = []
            if r0.boxes is not None and len(r0.boxes) > 0:
                boxes = r0.boxes
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clses = boxes.cls.cpu().numpy()
                names = r0.names  # dict 또는 list

                for (x1, y1, x2, y2), sc, cid in zip(xyxy, confs, clses):
                    label = names[cid] if isinstance(names, (list, tuple)) else names.get(int(cid), str(int(cid)))
                    if sc >= CONF_TH:
                        dets.append({
                            "label": label,
                            "conf": float(sc),
                            "bbox_xyxy": [int(x1), int(y1), int(x2), int(y2)]
                        })

            # ---- 쿨다운 후에만 발행 ----
            now = time.time()
            if dets and (now - last_pub_ts) >= ALERT_COOLDOWN:
                payload = {"dets": dets}  # ★ 서브스크라이버 포맷에 맞춤
                ros_msg = String()
                ros_msg.data = json.dumps(payload, ensure_ascii=False)
                pub.publish(ros_msg)
                #print(f"📢 Published /detections_json: {ros_msg.data}")
                last_pub_ts = now

            # ---- 시각화 ----
            annotated = r0.plot()
            cv2.imshow("IP Camera AI Inference", annotated)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 종료
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
