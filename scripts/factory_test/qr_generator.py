# -*- coding: utf-8 -*-
"""Generate QR images for connected factory-test devices."""

import os
import re
import datetime as _dt
from typing import Optional

from .constants import DEFAULT_QR_DIR
from .deps import QRCode, QRCODE_IMPORT_ERROR, qrcode

QR_IMAGE_SIZE = 300
_UNSAFE_FILENAME_CHARS = re.compile(r"[^0-9A-Za-z._-]+")
_HEX_PREFIX = re.compile(r"(?i)0x")


def _safe_filename_part(value: str) -> str:
    safe = _UNSAFE_FILENAME_CHARS.sub("_", value.strip())
    return safe.strip("._") or "unknown"


def _qr_payload(manufacturer_data: str) -> str:
    """Remove 0x prefixes from the value encoded in the QR code."""
    return _HEX_PREFIX.sub("", (manufacturer_data or "").strip())


def save_manufacturer_qr(
    manufacturer_data: str,
    output_dir: str = DEFAULT_QR_DIR,
    size: int = QR_IMAGE_SIZE,
) -> Optional[str]:
    """Save manufacturer data as a fixed-size PNG and return its path."""
    value = (manufacturer_data or "").strip()
    if not value:
        return None
    if QRCODE_IMPORT_ERROR is not None:
        raise RuntimeError(f"qrcode/Pillow 未安装或导入失败: {QRCODE_IMPORT_ERROR}")
    if size <= 0:
        raise ValueError("二维码尺寸必须大于 0")

    os.makedirs(output_dir, exist_ok=True)
    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{timestamp}_qr_{_safe_filename_part(value)}.png"
    path = os.path.join(output_dir, filename)

    qr = QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=10,
        border=4,
    )
    qr.add_data(_qr_payload(value))
    qr.make(fit=True)
    image = qr.make_image(fill_color="black", back_color="white").convert("RGB")

    # Resize after rendering so every generated file is exactly size x size.
    image.resize((size, size), 0).save(path, format="PNG")
    return path
