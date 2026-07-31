# -*- coding: utf-8 -*-
"""Optional third-party dependencies used by the factory test tool."""

import os
import sys


def _add_conda_dll_paths():
    candidates = [
        os.path.join(sys.prefix, "Library", "bin"),
        os.path.join(sys.prefix, "bin"),
    ]
    for path in candidates:
        if not os.path.isdir(path):
            continue
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(path)
            except Exception:
                pass
        os.environ["PATH"] = path + os.pathsep + os.environ.get("PATH", "")


_add_conda_dll_paths()

try:
    from bleak import BleakClient, BleakScanner
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    BleakClient = None
    BleakScanner = None
    BLEAK_IMPORT_ERROR = ex
else:
    BLEAK_IMPORT_ERROR = None

try:
    from openpyxl import Workbook
    from openpyxl.styles import Font, PatternFill
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    Workbook = None
    Font = None
    PatternFill = None
    OPENPYXL_IMPORT_ERROR = ex
else:
    OPENPYXL_IMPORT_ERROR = None

try:
    import cv2
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    cv2 = None
    CV2_IMPORT_ERROR = ex
else:
    CV2_IMPORT_ERROR = None

try:
    from pyzbar.pyzbar import decode as decode_qr
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    decode_qr = None
    PYZBAR_IMPORT_ERROR = ex
else:
    PYZBAR_IMPORT_ERROR = None
