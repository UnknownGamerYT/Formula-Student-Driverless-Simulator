#!/usr/bin/env python3
from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a fine-tuned YOLO detector.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--format", default="onnx", choices=["onnx", "engine", "torchscript", "openvino"])
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--int8", action="store_true")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    output = model.export(format=args.format, imgsz=args.imgsz, half=args.half, int8=args.int8)
    print(output)


if __name__ == "__main__":
    main()
