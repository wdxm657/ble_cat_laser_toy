# -*- coding: utf-8 -*-
"""Arrange generated QR images in a Markdown file for A4 printing.

Example:
    python make_qr_print_md.py
    python make_qr_print_md.py --input-dir qrcodes --output qr_print.md
"""

import argparse
import html
import os
from pathlib import Path
from typing import Iterable, List

# Ten columns fit comfortably on an A4 portrait page. Keep all rows in one
# table so Markdown-to-PDF converters do not insert a page break per row.
IMAGES_PER_ROW = 10
# Keep the HTML image dimensions explicit because some Markdown-to-PDF
# converters ignore CSS rules and otherwise use the source image size.
# 70px is small enough to leave only a narrow margin while keeping the code
# readable when printed.
QR_IMAGE_PX = 70


def _image_path_for_markdown(image_path: Path, output_path: Path) -> str:
    return Path(os.path.relpath(image_path, output_path.parent)).as_posix()


def _chunks(items: List[Path], size: int) -> Iterable[List[Path]]:
    for start in range(0, len(items), size):
        yield items[start:start + size]


def build_markdown(input_dir: Path, output_path: Path) -> str:
    images = sorted(
        path for path in input_dir.iterdir()
        if path.is_file() and path.suffix.lower() in {".png", ".jpg", ".jpeg"}
    )

    lines = [
        "<style>",
        "@page { size: A4 portrait; margin: 4mm; }",
        "body { margin: 0; padding: 0; }",
        ".qr-table { width: 100%; border-collapse: collapse; table-layout: fixed; }",
        ".qr-table tr { page-break-inside: avoid; break-inside: avoid; }",
        ".qr-table td { width: 10%; padding: 0; text-align: center; vertical-align: top; }",
        "</style>",
        "",
    ]

    if not images:
        lines.append("未找到二维码图片。")
        return "\n".join(lines) + "\n"

    lines.append('<table class="qr-table" style="width: 100%; table-layout: fixed; border-collapse: collapse;">')
    for group in _chunks(images, IMAGES_PER_ROW):
        lines.append("<tr>")
        for image_path in group:
            relative_path = html.escape(_image_path_for_markdown(image_path, output_path), quote=True)
            lines.append(
                f'<td style="width: 10%; padding: 0;">'
                f'<img src="{relative_path}" alt="" width="{QR_IMAGE_PX}" height="{QR_IMAGE_PX}" '
                f'style="width: {QR_IMAGE_PX}px; height: {QR_IMAGE_PX}px; display: block; margin: 0 auto;"></td>'
            )
        for _ in range(IMAGES_PER_ROW - len(group)):
            lines.append('<td style="width: 10%; padding: 0;"></td>')
        lines.append("</tr>")
    lines.extend(["</table>", ""])

    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="将二维码图片排版为 A4 打印用 Markdown")
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "qrcodes",
        # default=Path(__file__).resolve().parent / "qrcode_images",
        help="二维码图片目录，默认是当前程序目录下的 qrcodes",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "qr_print.md",
        help="输出 Markdown 文件，默认是当前程序目录下的 qr_print.md",
    )
    args = parser.parse_args()

    args.input_dir = args.input_dir.resolve()
    args.output = args.output.resolve()
    if not args.input_dir.is_dir():
        parser.error(f"二维码目录不存在: {args.input_dir}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_markdown(args.input_dir, args.output), encoding="utf-8")
    print(f"已生成: {args.output}")


if __name__ == "__main__":
    main()
