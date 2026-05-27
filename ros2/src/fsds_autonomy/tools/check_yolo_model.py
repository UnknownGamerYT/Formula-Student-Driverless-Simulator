#!/usr/bin/env python3
from __future__ import annotations

import argparse

from fsds_autonomy.constants import CONE_CLASS_TO_COLOR


def main() -> None:
    parser = argparse.ArgumentParser(description="Check whether YOLO weights expose cone classes.")
    parser.add_argument("model", help="Path/name of a YOLO .pt model")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    names = getattr(model, "names", {}) or {}
    cone_classes = {
        int(index): str(name)
        for index, name in names.items()
        if str(name) in CONE_CLASS_TO_COLOR or "cone" in str(name).lower()
    }

    print(f"model: {args.model}")
    print(f"class_count: {len(names)}")
    if cone_classes:
        print("status: OK cone classes found")
        for index, name in cone_classes.items():
            print(f"  {index}: {name}")
    else:
        preview = ", ".join(str(name) for _, name in list(names.items())[:12])
        print("status: NOT_A_CONE_MODEL")
        print(f"class_preview: {preview}")
        print("expected classes include: yellow_cone, blue_cone, orange_cone, large_orange_cone, unknown_cone")


if __name__ == "__main__":
    main()
