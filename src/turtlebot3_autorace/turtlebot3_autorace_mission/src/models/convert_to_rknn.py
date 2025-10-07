#!/usr/bin/env python3
"""Convert a YOLOv8 ONNX model into RKNN format and prepare a calibration dataset."""

from __future__ import annotations

import random
from pathlib import Path
from typing import Dict, Iterable, List

from rknn.api import RKNN


# ----------------- 경로 설정 -----------------
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parents[4]
PROJECT_ROOT = WORKSPACE_DIR
YOLO_LEARN_DIR = PROJECT_ROOT / "yolo_learn"

ROAD_ACCIDENT_DIR = YOLO_LEARN_DIR / "road_accident"
SINKHOLE_DIR = YOLO_LEARN_DIR / "sinkhole"

IMAGE_DIRECTORIES: List[Path] = [
    ROAD_ACCIDENT_DIR / "train" / "images",
    ROAD_ACCIDENT_DIR / "valid" / "images",
    SINKHOLE_DIR / "train" / "images",
    SINKHOLE_DIR / "valid" / "images",
]

DATASET_PATH = SCRIPT_DIR / "dataset_random.txt"
ONNX_MODEL = SCRIPT_DIR / "best_yolov8n.onnx"
RKNN_MODEL_INT8 = SCRIPT_DIR / "best_yolov8n_int8.rknn"
RKNN_MODEL_FP16 = SCRIPT_DIR / "best_yolov8n_fp16.rknn"

TARGET_PLATFORM = "rk3588"
TOTAL_SAMPLES = 200  # 생성할 샘플 수 (필요 시 변경)
# -------------------------------------------


def collect_image_files(image_dirs: Iterable[Path]) -> Dict[Path, List[Path]]:
    allowed_extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    collected: Dict[Path, List[Path]] = {}

    for directory in image_dirs:
        files: List[Path] = []
        if not directory.exists():
            print(f"Warning: Image directory not found: {directory}")
            collected[directory] = files
            continue

        for path in directory.rglob("*"):
            if path.is_file() and path.suffix.lower() in allowed_extensions:
                files.append(path.resolve())

        if not files:
            print(f"Warning: No supported image files found in {directory}")

        collected[directory] = files

    return collected


def create_random_dataset_file(image_dirs: Iterable[Path], output_path: Path, total_samples: int | None = None) -> bool:
    print("--> Creating dataset file from multiple directories...")
    per_directory_files = collect_image_files(image_dirs)

    available = [path for files in per_directory_files.values() for path in files]
    if not available:
        print("Error: No images collected from the provided directories.")
        return False

    selected: List[Path] = []
    remaining = None if total_samples is None or total_samples <= 0 else total_samples

    valid_dirs = [files for files in per_directory_files.values() if files]
    if remaining is not None and valid_dirs:
        share = max(1, remaining // len(valid_dirs))
        for files in valid_dirs:
            take = min(share, len(files))
            selected.extend(random.sample(files, take))
        remaining -= len(selected)

    if remaining is None or remaining > 0:
        pool = [path for path in available if path not in selected]
        if remaining is None or remaining > len(pool):
            selected.extend(pool)
        else:
            selected.extend(random.sample(pool, remaining))

    selected = sorted(set(selected))

    with output_path.open("w", encoding="utf-8") as handle:
        for path in selected:
            handle.write(str(path) + "\n")

    print(f"--> Wrote {len(selected)} image paths to {output_path}")
    return True


def convert_to_int8(onnx_path: Path, rknn_path: Path, dataset_path: Path, target_platform: str) -> None:
    print(f"--> Starting INT8 conversion for {onnx_path}")
    rknn = RKNN(verbose=True)

    print("--> Config model")
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=target_platform,
    )
    print("done")

    print("--> Loading model")
    ret = rknn.load_onnx(model=str(onnx_path))
    if ret != 0:
        print("Error: Load ONNX failed!")
        return

    print("--> Building model")
    ret = rknn.build(do_quantization=True, dataset=str(dataset_path))
    if ret != 0:
        print("Error: Build model failed!")
        return

    print("--> Exporting RKNN model")
    ret = rknn.export_rknn(str(rknn_path))
    if ret != 0:
        print("Error: Export RKNN failed!")
        return

    print(f"--> INT8 RKNN model saved to {rknn_path}")
    rknn.release()


def convert_to_fp16(onnx_path: Path, rknn_path: Path, target_platform: str) -> None:
    print(f"\n--> Starting FP16 conversion for {onnx_path}")
    rknn = RKNN(verbose=True)

    print("--> Config model")
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=target_platform,
    )
    print("done")

    print("--> Loading model")
    ret = rknn.load_onnx(model=str(onnx_path))
    if ret != 0:
        print("Error: Load ONNX failed!")
        return

    print("--> Building model")
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print("Error: Build model failed!")
        return

    print("--> Exporting RKNN model")
    ret = rknn.export_rknn(str(rknn_path))
    if ret != 0:
        print("Error: Export RKNN failed!")
        return

    print(f"--> FP16 RKNN model saved to {rknn_path}")
    rknn.release()


def main() -> None:
    created = create_random_dataset_file(IMAGE_DIRECTORIES, DATASET_PATH, TOTAL_SAMPLES)
    if not created:
        print("Dataset generation failed; aborting quantized conversion.")
    else:
        convert_to_int8(ONNX_MODEL, RKNN_MODEL_INT8, DATASET_PATH, TARGET_PLATFORM)

    convert_to_fp16(ONNX_MODEL, RKNN_MODEL_FP16, TARGET_PLATFORM)


if __name__ == "__main__":
    main()
