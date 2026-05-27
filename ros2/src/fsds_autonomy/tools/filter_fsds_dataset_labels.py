#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

import cv2

from fsds_autonomy.constants import COLOR_TO_CONE_CLASS
from fsds_autonomy.label_quality import refine_projected_label_with_color, yolo_label_to_bbox


def parse_label(line: str) -> tuple[int, float, float, float, float] | None:
    parts = line.strip().split()
    if len(parts) != 5:
        return None
    try:
        return int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
    except ValueError:
        return None


def write_dataset_yaml(dataset_dir: Path) -> None:
    names = [COLOR_TO_CONE_CLASS[index] for index in sorted(COLOR_TO_CONE_CLASS)]
    text = "\n".join(
        [
            f"path: {dataset_dir}",
            "train: images",
            "val: images",
            "names:",
            *[f"  {i}: {name}" for i, name in enumerate(names)],
            "",
        ]
    )
    (dataset_dir / "fsds_cones.yaml").write_text(text, encoding="utf-8")


def draw_preview(dataset_dir: Path, out_path: Path, samples: int = 12) -> None:
    image_paths = sorted((dataset_dir / "images").glob("*.jpg"))
    if not image_paths:
        return
    selected = random.sample(image_paths, min(samples, len(image_paths)))
    tiles = []
    for image_path in selected:
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        height, width = frame.shape[:2]
        label_path = dataset_dir / "labels" / f"{image_path.stem}.txt"
        if label_path.exists():
            for line in label_path.read_text(encoding="utf-8").splitlines():
                label = parse_label(line)
                if label is None:
                    continue
                color, *_ = label
                x1, y1, x2, y2 = yolo_label_to_bbox(label, width, height)
                if color == 0:
                    draw_color = (0, 255, 255)
                elif color == 1:
                    draw_color = (255, 80, 0)
                elif color in (2, 3):
                    draw_color = (0, 140, 255)
                else:
                    draw_color = (180, 180, 180)
                cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)
        cv2.putText(frame, image_path.stem, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(cv2.resize(frame, (420, 315), interpolation=cv2.INTER_AREA))
    if not tiles:
        return
    columns = 3
    rows = (len(tiles) + columns - 1) // columns
    blank = tiles[0] * 0
    while len(tiles) < rows * columns:
        tiles.append(blank.copy())
    row_images = [cv2.hconcat(tiles[index * columns : (index + 1) * columns]) for index in range(rows)]
    sheet = cv2.vconcat(row_images)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), sheet)


def main() -> None:
    parser = argparse.ArgumentParser(description="Drop or re-center synthetic FSDS YOLO labels that disagree with image color.")
    parser.add_argument("dataset", help="Dataset directory containing images/ and labels/")
    parser.add_argument("--out", help="Filtered dataset directory. Defaults to <dataset>_color_checked")
    parser.add_argument("--search-scale", type=float, default=2.6)
    parser.add_argument("--min-color-pixels", type=int, default=5)
    parser.add_argument("--min-color-ratio", type=float, default=0.012)
    parser.add_argument("--no-snap", action="store_true", help="Only reject labels; do not re-center boxes.")
    parser.add_argument("--preview", action="store_true", help="Write label_preview_contact_sheet.jpg in the output dataset.")
    args = parser.parse_args()

    dataset_dir = Path(args.dataset).expanduser()
    out_dir = Path(args.out).expanduser() if args.out else dataset_dir.with_name(f"{dataset_dir.name}_color_checked")
    image_out = out_dir / "images"
    label_out = out_dir / "labels"
    image_out.mkdir(parents=True, exist_ok=True)
    label_out.mkdir(parents=True, exist_ok=True)

    image_count = 0
    kept_image_count = 0
    label_count = 0
    kept_label_count = 0
    shifted_label_count = 0
    dropped_label_count = 0

    for image_path in sorted((dataset_dir / "images").glob("*.jpg")):
        label_path = dataset_dir / "labels" / f"{image_path.stem}.txt"
        if not label_path.exists():
            continue
        image_count += 1
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        kept_labels: list[tuple[int, float, float, float, float]] = []
        for line in label_path.read_text(encoding="utf-8").splitlines():
            label = parse_label(line)
            if label is None:
                continue
            label_count += 1
            refined = refine_projected_label_with_color(
                frame,
                label,
                search_scale=args.search_scale,
                min_color_pixels=args.min_color_pixels,
                min_color_ratio=args.min_color_ratio,
                snap_to_color=not args.no_snap,
            )
            if refined is None:
                dropped_label_count += 1
                continue
            refined_label, stats = refined
            kept_labels.append(refined_label)
            kept_label_count += 1
            if stats.shift_px > 2.0:
                shifted_label_count += 1
        if not kept_labels:
            continue
        kept_image_count += 1
        shutil.copy2(image_path, image_out / image_path.name)
        (label_out / label_path.name).write_text(
            "\n".join(f"{color} {x:.6f} {y:.6f} {w:.6f} {h:.6f}" for color, x, y, w, h in kept_labels) + "\n",
            encoding="utf-8",
        )

    write_dataset_yaml(out_dir)
    if args.preview:
        draw_preview(out_dir, out_dir / "label_preview_contact_sheet.jpg")

    print(f"input_images: {image_count}")
    print(f"kept_images: {kept_image_count}")
    print(f"input_labels: {label_count}")
    print(f"kept_labels: {kept_label_count}")
    print(f"dropped_labels: {dropped_label_count}")
    print(f"shifted_labels_over_2px: {shifted_label_count}")
    print(f"output: {out_dir}")


if __name__ == "__main__":
    main()
