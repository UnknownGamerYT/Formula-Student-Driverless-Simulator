#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def export_with_fallback(model, kwargs: dict) -> str:
    try:
        return str(model.export(**kwargs))
    except TypeError as exc:
        minimal_keys = {"format", "imgsz", "half", "int8", "device"}
        minimal_kwargs = {key: value for key, value in kwargs.items() if key in minimal_keys}
        if minimal_kwargs == kwargs:
            raise
        print(f"Export retrying without optional arguments unsupported by this Ultralytics version: {exc}")
        return str(model.export(**minimal_kwargs))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Export a fine-tuned YOLO detector for the FSDS camera detector. "
            "Use --format onnx for portable inference or --format engine for TensorRT on this GPU."
        )
    )
    parser.add_argument("--model", required=True, help="Path to trained .pt weights or another Ultralytics model.")
    parser.add_argument("--format", default="onnx", choices=["onnx", "engine", "torchscript", "openvino"])
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--device", default="auto", help="Ultralytics device, for example auto, cpu, 0, cuda:0.")
    parser.add_argument("--half", action="store_true", help="Export FP16 where supported. Recommended for TensorRT.")
    parser.add_argument("--int8", action="store_true", help="Export INT8 where supported. Needs calibration data.")
    parser.add_argument("--dynamic", action="store_true", help="Export dynamic input shapes where supported.")
    parser.add_argument("--batch", type=int, default=1, help="Static export batch size.")
    parser.add_argument("--workspace", type=float, default=None, help="TensorRT workspace size in GiB where supported.")
    parser.add_argument("--opset", type=int, default=None, help="ONNX opset version.")
    parser.add_argument("--nms", action="store_true", help="Add NMS to exported graph where supported.")
    simplify = parser.add_mutually_exclusive_group()
    simplify.add_argument("--simplify", dest="simplify", action="store_true", default=None)
    simplify.add_argument("--no-simplify", dest="simplify", action="store_false")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    kwargs = {
        "format": args.format,
        "imgsz": args.imgsz,
        "half": args.half,
        "int8": args.int8,
        "batch": args.batch,
    }
    if args.device.lower() not in {"", "auto", "none"}:
        kwargs["device"] = args.device
    if args.dynamic:
        kwargs["dynamic"] = True
    if args.workspace is not None:
        kwargs["workspace"] = args.workspace
    if args.opset is not None:
        kwargs["opset"] = args.opset
    if args.nms:
        kwargs["nms"] = True
    if args.simplify is not None:
        kwargs["simplify"] = args.simplify

    output = export_with_fallback(model, kwargs)
    output_path = Path(output).expanduser()
    print(output_path)


if __name__ == "__main__":
    main()
