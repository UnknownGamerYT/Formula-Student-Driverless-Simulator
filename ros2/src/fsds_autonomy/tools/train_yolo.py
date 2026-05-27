#!/usr/bin/env python3
from __future__ import annotations

import argparse


def parse_batch(value: str) -> int | float:
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("batch must be an int or float, for example 16 or 0.60") from exc


def parse_cache(value: str) -> bool | str:
    normalized = value.strip().lower()
    if normalized in {"false", "0", "no", "off", "none"}:
        return False
    if normalized in {"true", "1", "yes", "on", "ram"}:
        return True
    if normalized == "disk":
        return "disk"
    raise argparse.ArgumentTypeError("cache must be false, true, ram, or disk")


def main() -> None:
    parser = argparse.ArgumentParser(description="Fine-tune a YOLO detector on FSDS cone data.")
    parser.add_argument("--data", required=True, help="Ultralytics dataset yaml")
    parser.add_argument("--model", required=True, help="Starting weights, e.g. yolo26n.pt")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=parse_batch, default=16)
    parser.add_argument("--device", default=0)
    parser.add_argument("--cache", type=parse_cache, default=False)
    parser.add_argument("--project", default="runs/fsds_cones")
    parser.add_argument("--name", default="yolo26_fsds")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        amp=True,
        cache=args.cache,
        plots=True,
    )


if __name__ == "__main__":
    main()
