import os, time, yaml, sys
import threading
import json
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    _ROS_AVAILABLE = True
except Exception:
    _ROS_AVAILABLE = False
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from collections import deque

# ================== 경로/설정 ==================
RTSP_URL = "rtsp://admin:admin%401234@10.10.16.201:554/0"
ENGINE   = "/home/otrin/intel7_final_ws/ai_model/best.engine"
META     = "/home/otrin/intel7_final_ws/ai_model/metadata.yaml"

IMG_SIZE = (640, 640)

# ===== 탐지/과검출/지속성 제어 =====
CONF_TH         = 0.6   # 기본 신뢰도 컷 (YOLOv8 4+nc면 cls 확률 그대로)
NMS_IOU         = 0.55   # NMS IoU (겹침 제거 강도)
MIN_WH          = 20     # 최소 한 변(px)
PRE_TOPK        = 800    # NMS 전 후보 상한 (속도/잡박스 제어)
CLASS_TOPK      = 5      # ★ 클래스별 최종 Top-K (장면당 상한)
GRID_CELL       = 16     # ★ 같은 위치 중복 억제(셀 크기, px)
DIoU_DIST_TH    = 0.20   # ★ 중심거리 억제(정규화 거리 임계)
MAX_DET         = 10     # 최종 최대 박스 수(상한)

# ===== 프레임 간 지속성 필터(트래킹 없이 가벼운 디바운서) =====
PERSIST_MIN_HITS = 2     # ★ 연속 hits가 이 값 이상이어야 화면에 표시
PERSIST_MAX_AGE  = 4     # 누락 허용 프레임 수(트래커 유지)
MATCH_IOU_TH     = 0.4   # 프레임간 매칭 IoU
MATCH_DIST_TH    = 0.10

# ===== ROS 퍼블리시 주기 =====
PUBLISH_INTERVAL_SEC = 5.0
  # 프레임간 매칭 중심거리(정규화)

# ================== 유틸 ==================
def sigmoid(x): return 1 / (1 + np.exp(-x))

def letterbox(img, new=(640, 640), color=(114,114,114)):
    h, w = img.shape[:2]
    r = min(new[0]/h, new[1]/w)
    nh, nw = int(round(h*r)), int(round(w*r))
    top, left = (new[0]-nh)//2, (new[1]-nw)//2
    if (w,h)!=(nw,nh):
        img = cv2.resize(img, (nw,nh), interpolation=cv2.INTER_LINEAR)
    img = cv2.copyMakeBorder(img, top, new[0]-nh-top, left, new[1]-nw-left,
                             cv2.BORDER_CONSTANT, value=color)
    return img, r, (left, top)

def to_chw_rgb01(bgr):
    x = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.transpose(x, (2,0,1))[None,...].copy(order='C')

# ================== 클래스 이름 ==================
if os.path.exists(META):
    with open(META) as f:
        meta = yaml.safe_load(f)
    nf = meta.get("names", [])
    NAMES = [nf[k] for k in sorted(nf)] if isinstance(nf, dict) else list(nf)
else:
    NAMES = ["object"]

# ================== 출력 디코드 ==================
def auto_decode(output_flat, oshape, meta_num_cls):
    raw = output_flat.reshape(-1)
    cand = []
    if len(oshape) == 3:
        b, c, n = oshape
        cand += [("1,C,N", raw.reshape(c, n).T),
                 ("1,N,C", raw.reshape(n, c))]
    elif len(oshape) == 2:
        a, b_ = oshape
        cand += [("(N,C)", raw.reshape(a, b_)),
                 ("(C,N)", raw.reshape(b_, a).T)]
    def fit_score(p):
        C = p.shape[1]; best_nc = max(1, meta_num_cls)
        return -min(abs((C-4)-best_nc), abs((C-5)-best_nc))
    best = None
    if cand:
        best = max(cand, key=lambda x: fit_score(x[1]))[1]
    if best is None or best.shape[1] < 6:
        for N_guess in (8400, 25200, 6300, 2100):
            if raw.size % N_guess == 0:
                C_guess = raw.size // N_guess
                if C_guess >= 6:
                    best = raw.reshape(N_guess, C_guess); break
    if best is None:
        best = raw.reshape(-1, 6)
    return best.astype(np.float32)

# ================== 상보 억제 유틸 ==================
def iou_xyxy(a, b):
    # a: [N,4], b:[M,4]
    xx1 = np.maximum(a[:,None,0], b[None,:,0])
    yy1 = np.maximum(a[:,None,1], b[None,:,1])
    xx2 = np.minimum(a[:,None,2], b[None,:,2])
    yy2 = np.minimum(a[:,None,3], b[None,:,3])
    w = np.maximum(0.0, xx2-xx1); h = np.maximum(0.0, yy2-yy1)
    inter = w*h
    area_a = (a[:,2]-a[:,0])*(a[:,3]-a[:,1])
    area_b = (b[:,2]-b[:,0])*(b[:,3]-b[:,1])
    return inter / (area_a[:,None] + area_b[None,:] - inter + 1e-6)

def center_dist_norm(a, b, W, H):
    # a: [N,4], b:[M,4]  -> normalized distance by image diagonal
    cx_a = 0.5*(a[:,0]+a[:,2]); cy_a = 0.5*(a[:,1]+a[:,3])
    cx_b = 0.5*(b[:,0]+b[:,2]); cy_b = 0.5*(b[:,1]+b[:,3])
    dx = (cx_a[:,None]-cx_b[None,:]); dy = (cy_a[:,None]-cy_b[None,:])
    dist = np.sqrt(dx*dx + dy*dy)
    diag = np.sqrt(W*W + H*H)
    return dist / (diag + 1e-6)

def diou_nms(xyxy, score, iou_th=0.55, dist_th=0.2, max_det=60):
    # IoU와 중심거리 둘 다 고려한 억제 (class-agnostic)
    order = score.argsort()[::-1]
    keep = []
    while order.size > 0 and len(keep) < max_det:
        i = order[0]; keep.append(i)
        if order.size == 1: break
        rest = order[1:]
        # IoU 억제
        xx1 = np.maximum(xyxy[i,0], xyxy[rest,0])
        yy1 = np.maximum(xyxy[i,1], xyxy[rest,1])
        xx2 = np.minimum(xyxy[i,2], xyxy[rest,2])
        yy2 = np.minimum(xyxy[i,3], xyxy[rest,3])
        w = np.maximum(0.0, xx2-xx1); h = np.maximum(0.0, yy2-yy1)
        inter = w*h
        area_i = (xyxy[i,2]-xyxy[i,0])*(xyxy[i,3]-xyxy[i,1])
        area_r = (xyxy[rest,2]-xyxy[rest,0])*(xyxy[rest,3]-xyxy[rest,1])
        iou = inter / (area_i + area_r - inter + 1e-6)
        # 중심 거리 (정규화 X, 여기선 상대성 위해 나중에 별도 조건)
        keep_mask = iou < iou_th
        order = rest[keep_mask]
    return np.array(keep, dtype=np.int32)

# ================== 후처리 (강화) ==================
def postprocess_strong(output_flat, oshape, W, H, r, pad, names):
    meta_nc = max(1, len(names))
    pred = auto_decode(output_flat, oshape, meta_nc)  # (N,C)
    N, C = pred.shape

    # obj 유무/클래스 판정
    cand_nc4 = C - 4
    cand_nc5 = C - 5
    has_obj = False; nc_rt = cand_nc4; cls_start = 4
    if cand_nc4 < 1 and cand_nc5 >= 1:
        has_obj = True;  nc_rt = cand_nc5; cls_start = 5
    elif cand_nc4 >= 1 and cand_nc5 >= 1:
        d4 = abs(cand_nc4 - meta_nc); d5 = abs(cand_nc5 - meta_nc)
        if d5 < d4:
            has_obj = True;  nc_rt = cand_nc5; cls_start = 5
        else:
            has_obj = False; nc_rt = cand_nc4; cls_start = 4
    nc_rt = max(1, int(nc_rt))

    # 좌표 복원
    box = pred[:, :4].astype(np.float32)
    if box.max() <= 2.0:
        box *= np.array([IMG_SIZE[1], IMG_SIZE[0], IMG_SIZE[1], IMG_SIZE[0]], np.float32)

    # 클래스 확률 (2클래스면 softmax가 겹침 억제에 유리)
    logits = pred[:, cls_start:cls_start+nc_rt].astype(np.float32)
    if logits.shape[1] >= 2:
        m = logits.max(axis=1, keepdims=True)
        ex = np.exp(logits - m)
        cls = ex / np.clip(ex.sum(axis=1, keepdims=True), 1e-9, None)
    else:
        cls = sigmoid(logits)
    cls_max = cls.max(axis=1); cls_id = cls.argmax(axis=1)

    # 점수 계산
    if has_obj:
        obj = sigmoid(pred[:, 4].astype(np.float32))
        score = obj * cls_max
    else:
        score = cls_max

    # Top-K 프리필터
    if PRE_TOPK and PRE_TOPK < score.size:
        top_idx = np.argpartition(-score, PRE_TOPK)[:PRE_TOPK]
        box, score, cls_id = box[top_idx], score[top_idx], cls_id[top_idx]

    # 1차 컷
    keep = score >= CONF_TH
    if not np.any(keep): return []
    box, score, cls_id = box[keep], score[keep], cls_id[keep]

    # xywh -> xyxy
    xyxy = np.empty_like(box)
    xyxy[:,0] = box[:,0]-box[:,2]/2; xyxy[:,1] = box[:,1]-box[:,3]/2
    xyxy[:,2] = box[:,0]+box[:,2]/2; xyxy[:,3] = box[:,1]+box[:,3]/2

    # letterbox 복원
    left, top = pad
    xyxy[:,[0,2]] -= left; xyxy[:,[1,3]] -= top
    xyxy /= r
    xyxy[:,0::2] = np.clip(xyxy[:,0::2], 0, W-1)
    xyxy[:,1::2] = np.clip(xyxy[:,1::2], 0, H-1)

    # 최소 크기
    wh = (xyxy[:,2:4]-xyxy[:,0:2]).clip(min=0)
    msz = np.minimum(wh[:,0], wh[:,1]) >= MIN_WH
    if not np.any(msz): return []
    xyxy, score, cls_id = xyxy[msz], score[msz], cls_id[msz]

    # ===== (A) 위치 격자 억제 (cell top-1) =====
    cx = 0.5*(xyxy[:,0]+xyxy[:,2]); cy = 0.5*(xyxy[:,1]+xyxy[:,3])
    gx = (cx / GRID_CELL).astype(np.int32)
    gy = (cy / GRID_CELL).astype(np.int32)
    cell_id = gx + 100000*gy
    keep_mask = np.zeros(len(score), dtype=bool)
    order = score.argsort()[::-1]
    seen = set()
    for i in order:
        k = int(cell_id[i])
        if k in seen: continue
        seen.add(k)
        keep_mask[i] = True
    xyxy, score, cls_id = xyxy[keep_mask], score[keep_mask], cls_id[keep_mask]

    # ===== (B) DIoU-NMS + 중심거리 억제 =====
    if xyxy.shape[0] == 0: return []
    # DIoU-NMS로 1차 정리
    keep_idx = diou_nms(xyxy, score, iou_th=NMS_IOU, dist_th=DIoU_DIST_TH, max_det=MAX_DET)
    xyxy, score, cls_id = xyxy[keep_idx], score[keep_idx], cls_id[keep_idx]

    # 중심거리 추가 억제(겹치지 않아도 너무 가까운 중복 제거)
    if xyxy.shape[0] > 1:
        D = center_dist_norm(xyxy, xyxy, W, H)
        np.fill_diagonal(D, 1.0)
        reserve = np.ones(len(score), dtype=bool)
        ord2 = score.argsort()[::-1]
        for idx_i in ord2:
            if not reserve[idx_i]: continue
            close = (D[idx_i] < DIoU_DIST_TH) & reserve
            close[idx_i] = False
            reserve[close] = False
        xyxy, score, cls_id = xyxy[reserve], score[reserve], cls_id[reserve]

    # ===== (C) 클래스별 Top-K 제한 =====
    if xyxy.shape[0] > 0 and CLASS_TOPK is not None:
        keep_final = []
        for c in np.unique(cls_id):
            idx_c = np.where(cls_id == c)[0]
            if idx_c.size > CLASS_TOPK:
                sub = idx_c[np.argsort(-score[idx_c])[:CLASS_TOPK]]
            else:
                sub = idx_c
            keep_final.append(sub)
        keep_final = np.concatenate(keep_final) if keep_final else np.array([], dtype=int)
        xyxy, score, cls_id = xyxy[keep_final], score[keep_final], cls_id[keep_final]

    # 이름 배열 길이 보정
    if len(names) != nc_rt:
        base = list(names)
        names[:] = (base + [f"cls{i}" for i in range(len(base), nc_rt)])[:nc_rt]

    # 점수 내림차순 정렬
    idx = np.argsort(-score)
    xyxy, score, cls_id = xyxy[idx], score[idx], cls_id[idx]
    return [(xyxy[i], float(score[i]), int(cls_id[i])) for i in range(xyxy.shape[0])]

# ================== 간단 트래커(지속성 필터) ==================
class SimplePersistence:
    def __init__(self):
        self.tracks = []  # list of dict: {box, cls, score, hits, age}
        self.next_id = 0

    @staticmethod
    def _iou(a, b):
        xx1 = max(a[0], b[0]); yy1 = max(a[1], b[1])
        xx2 = min(a[2], b[2]); yy2 = min(a[3], b[3])
        w = max(0.0, xx2-xx1); h = max(0.0, yy2-yy1)
        inter = w*h
        area_a = (a[2]-a[0])*(a[3]-a[1])
        area_b = (b[2]-b[0])*(b[3]-b[1])
        return inter / (area_a + area_b - inter + 1e-6)

    @staticmethod
    def _center_dist_norm(a, b, W, H):
        cx1 = 0.5*(a[0]+a[2]); cy1 = 0.5*(a[1]+a[3])
        cx2 = 0.5*(b[0]+b[2]); cy2 = 0.5*(b[1]+b[3])
        d = np.hypot(cx1-cx2, cy1-cy2)
        return d / (np.hypot(W, H) + 1e-6)

    def update(self, dets, W, H):
        # dets: list[(xyxy, score, cid)]
        # 1) 모든 track age+1
        for t in self.tracks:
            t["age"] += 1

        used = set()
        # 2) 매칭(그리디): IoU 또는 중심거리로 연결
        for (xyxy, sc, cid) in dets:
            best_j = -1; best_cost = 1e9
            for j, t in enumerate(self.tracks):
                if t["cls"] != cid: continue
                iou = self._iou(t["box"], xyxy)
                dist = self._center_dist_norm(t["box"], xyxy, W, H)
                if iou >= MATCH_IOU_TH or dist <= MATCH_DIST_TH:
                    # 비용 낮을수록 좋고, IoU를 우선
                    cost = (1 - iou) + dist
                    if cost < best_cost:
                        best_cost, best_j = cost, j
            if best_j >= 0:
                # 매칭 성공 → 갱신
                t = self.tracks[best_j]
                t["box"] = xyxy
                t["score"] = max(t["score"], sc)
                t["hits"] += 1
                t["age"] = 0
                used.add(best_j)
            else:
                # 신규 트랙
                self.tracks.append({"id": self.next_id, "box": xyxy, "cls": cid,
                                    "score": sc, "hits": 1, "age": 0})
                self.next_id += 1

        # 3) 오래된(미갱신) 트랙 제거
        self.tracks = [t for t in self.tracks if t["age"] <= PERSIST_MAX_AGE]

        # 4) 표시에 쓸 트랙만 반환(연속 hits 기준)
        show = []
        for t in self.tracks:
            if t["hits"] >= PERSIST_MIN_HITS:
                show.append((t["box"], t["score"], t["cls"]))
        return show

# ================== 프레임 소스 (저지연) ==================
class LatestFrameStream:
    def __init__(self, cap, buffer_size=2, wait_sleep=0.002):
        self.cap = cap
        self.frames = deque(maxlen=max(1, buffer_size))
        self.wait_sleep = float(wait_sleep)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def _reader(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frames.append(frame)

    def read(self, timeout=0.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if self.frames:
                    frame = self.frames[-1]
                    self.frames.clear()
                    return True, frame
            if not self.running:
                break
            time.sleep(self.wait_sleep)
        return False, None

# ================== TensorRT 로드 ==================
TRT_LOGGER = trt.Logger(trt.Logger.INFO)
with open(ENGINE, "rb") as f, trt.Runtime(TRT_LOGGER) as rt:
    engine = rt.deserialize_cuda_engine(f.read())
ctx = engine.create_execution_context()

tensor_names = [n for n in engine]
inputs  = [n for n in tensor_names if engine.get_tensor_mode(n)==trt.TensorIOMode.INPUT]
outputs = [n for n in tensor_names if engine.get_tensor_mode(n)==trt.TensorIOMode.OUTPUT]
inp = inputs[0]
in_shape = engine.get_tensor_shape(inp)

# 디바이스 메모리/바인딩 준비
d_ptr, h_out = {}, {}
for n in tensor_names:
    shape = engine.get_tensor_shape(n)
    n_elem = abs(int(np.prod(shape)))
    d_ptr[n] = cuda.mem_alloc(n_elem * 4)
    ctx.set_tensor_address(n, int(d_ptr[n]))
    if n in outputs:
        h_out[n] = cuda.pagelocked_empty(n_elem, dtype=np.float32)
bindings = [int(d_ptr[n]) for n in tensor_names]

print(f"✅ TensorRT 엔진 로드 | 입력:{in_shape} | 출력수:{len(outputs)} | TRT_NMS:False")
print("출력 텐서:", outputs)

# ================== RTSP 오픈 ==================
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tcp|stimeout;5000000|fflags;nobuffer|flags;low_delay|"
    "use_wallclock_as_timestamps;1|max_delay;0|buffer_size;0"
)
cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print("❌ IP 카메라 연결 실패"); sys.exit(1)
if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

cv2.namedWindow("TRT Inference", cv2.WINDOW_NORMAL)
cv2.resizeWindow("TRT Inference", 1280, 720)
colors = np.random.randint(0,255,(max(1, len(NAMES)),3),dtype=np.uint8)

# 지속성 필터
persist = SimplePersistence()

frame_stream = LatestFrameStream(cap)
frame_stream.start()

# ROS 2 퍼블리셔 준비
_ros_node = None
_ros_pub = None
if _ROS_AVAILABLE:
    try:
        rclpy.init(args=None)
        _ros_node = rclpy.create_node('camera_pub')
        _ros_pub = _ros_node.create_publisher(String, 'detections_json', 10)
        print('ROS2 publisher ready: /detections_json (std_msgs/String)')
    except Exception as e:
        print(f'ROS2 init failed: {e}')
        _ROS_AVAILABLE = False


# ================== 메인 루프 ==================
prev = 0
warned_timeout = False
last_pub = 0.0
try:
    while True:
        ok, frame = frame_stream.read(timeout=0.5)
        if not ok:
            if not warned_timeout:
                print("⚠️ 프레임 대기 타임아웃")
                warned_timeout = True
            continue
        warned_timeout = False

        H, W = frame.shape[:2]
        img, r, pad = letterbox(frame, IMG_SIZE)
        blob = to_chw_rgb01(img)

        # 추론
        cuda.memcpy_htod(d_ptr[inp], blob)
        ok = ctx.execute_v2(bindings)
        if not ok:
            print("❌ TRT 실행 실패")
            break

        out_name = outputs[0]
        cuda.memcpy_dtoh(h_out[out_name], d_ptr[out_name])

        # 강화 후처리
        dets = postprocess_strong(
            h_out[out_name], engine.get_tensor_shape(out_name),
            W, H, r, pad, names=NAMES
        )

        # ===== 프레임 간 지속성 필터 적용 =====
        dets_show = persist.update(dets, W, H)

        # ROS 2 발행 (JSON)
        if _ros_pub is not None:
            pub_now = time.time()
            if pub_now - last_pub >= PUBLISH_INTERVAL_SEC:
                try:
                    payload_dets = []
                    for (xyxy, sc, cid) in dets_show:
                        x1, y1, x2, y2 = map(int, xyxy)
                        name = NAMES[cid] if cid < len(NAMES) else f'id{cid}'
                        payload_dets.append({
                            'xyxy': [x1, y1, x2, y2],
                            'score': float(sc),
                            'cls': int(cid),
                            'name': name,
                        })
                    payload = {'w': int(W), 'h': int(H), 'dets': payload_dets}
                    msg = String()
                    msg.data = json.dumps(payload, ensure_ascii=False)
                    _ros_pub.publish(msg)
                    last_pub = pub_now
                except Exception as e:
                    # 발행 실패는 치명적이지 않으므로 경고만 출력
                    print(f'ROS2 publish error: {e}')

        # 시각화
        for (xyxy, sc, cid) in dets_show:
            x1, y1, x2, y2 = map(int, xyxy)
            color = [int(v) for v in colors[cid % max(1, len(NAMES))]]
            name = NAMES[cid] if cid < len(NAMES) else f"id{cid}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{name} {sc:.2f}", (x1, max(22, y1-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # FPS
        now = time.time()
        fps = 1 / (now - prev) if prev > 0 else 0
        prev = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("TRT Inference", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
finally:
    frame_stream.stop()
    cap.release()
    cv2.destroyAllWindows()
    if _ros_node is not None:
        try:
            _ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
