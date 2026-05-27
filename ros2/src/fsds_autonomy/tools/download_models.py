#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Cache detector starting weights for FSDS autonomy.")
    parser.add_argument("--out", default="models/pretrained", help="Output model cache directory")
    parser.add_argument(
        "--rc-model",
        default="/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt",
        help="Existing RC model to copy if present",
    )
    parser.add_argument("--ultralytics", nargs="*", default=["yolo26n.pt", "yolo26s.pt"], help="Ultralytics model names to fetch")
    args = parser.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    rc_model = Path(args.rc_model).expanduser()
    if rc_model.exists():
        target = out_dir / rc_model.name
        shutil.copy2(rc_model, target)
        print(f"copied {rc_model} -> {target}")

    try:
        from ultralytics import YOLO
    except Exception:
        print("ultralytics is not installed; run: python3 -m pip install -r ros2/src/fsds_autonomy/requirements-ml.txt")
        return

    for model_name in args.ultralytics:
        try:
            model = YOLO(model_name)
            source = Path(getattr(model, "ckpt_path", "") or model_name)
            if source.exists():
                target = out_dir / source.name
                if source.resolve() != target.resolve():
                    shutil.copy2(source, target)
                print(f"cached {target}")
            else:
                print(f"loaded {model_name}; ultralytics cache owns the file")
        except Exception as exc:
            print(f"could not fetch {model_name}: {exc}")


if __name__ == "__main__":
    main()
