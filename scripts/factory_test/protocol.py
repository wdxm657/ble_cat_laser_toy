# -*- coding: utf-8 -*-
"""BLE factory-test protocol helpers."""

import uuid
from typing import Dict, Optional, Set

from .constants import (
    CTRL_MSG_TYPE_CMD,
    CTRL_MSG_TYPE_EVENT,
    CTRL_MSG_TYPE_RSP,
    CTRL_PROTO_VERSION,
    CTRL_STATUS_TEXT,
)


def uuid_candidates(raw: bytes) -> Set[int]:
    return {uuid.UUID(bytes=raw).int, uuid.UUID(bytes=raw[::-1]).int}


def char_uuid_int(char) -> Optional[int]:
    try:
        return uuid.UUID(str(char.uuid)).int
    except Exception:
        return None


def find_characteristic(client, raw_uuid: bytes):
    targets = uuid_candidates(raw_uuid)
    for service in client.services:
        for char in service.characteristics:
            if char_uuid_int(char) in targets:
                return char
    return None


def format_manufacturer_data(mfr: Dict[int, bytes]) -> str:
    if not mfr:
        return ""
    parts = []
    for company_id, data in sorted(mfr.items()):
        raw = int(company_id).to_bytes(2, "little") + bytes(data)
        parts.append("0x" + raw.hex().upper())
    return "; ".join(parts)


def normalize_mfr_value(value: str) -> str:
    return "".join(ch for ch in (value or "").upper() if ch in "0123456789ABCDEF")


def mfr_matches(qr_value: str, manufacturer_data: str) -> bool:
    qr_norm = normalize_mfr_value(qr_value)
    if not qr_norm:
        return False
    for part in (manufacturer_data or "").split(";"):
        part_norm = normalize_mfr_value(part)
        if _normalized_mfr_matches(qr_norm, part_norm):
            return True
    return _normalized_mfr_matches(qr_norm, normalize_mfr_value(manufacturer_data))


def _normalized_mfr_matches(left: str, right: str) -> bool:
    if not left or not right:
        return False
    if left == right:
        return True
    if min(len(left), len(right)) >= 8:
        return left in right or right in left
    return False


def build_ctrl_cmd(cmd_id: int, seq: int, payload: bytes = b"") -> bytes:
    plen = len(payload)
    return bytes(
        [
            CTRL_PROTO_VERSION,
            CTRL_MSG_TYPE_CMD,
            cmd_id & 0xFF,
            seq & 0xFF,
            plen & 0xFF,
            (plen >> 8) & 0xFF,
        ]
    ) + payload


def decode_ctrl_frame(data: bytes) -> Dict[str, object]:
    if len(data) < 6:
        return {
            "short": True,
            "raw": data,
            "text": f"CTRL short: {data.hex(' ').upper()}",
        }
    version, msg_type, cmd_id, seq = data[0], data[1], data[2], data[3]
    payload_len = data[4] | (data[5] << 8)
    payload = data[6 : 6 + payload_len]
    type_name = {CTRL_MSG_TYPE_CMD: "CMD", CTRL_MSG_TYPE_RSP: "RSP", CTRL_MSG_TYPE_EVENT: "EVT"}.get(msg_type, f"0x{msg_type:02X}")
    status = None
    if msg_type == CTRL_MSG_TYPE_RSP and payload:
        status = CTRL_STATUS_TEXT.get(payload[0], f"0x{payload[0]:02X}")
    return {
        "short": False,
        "raw": data,
        "version": version,
        "msg_type": msg_type,
        "cmd_id": cmd_id,
        "seq": seq,
        "payload_len": payload_len,
        "payload": payload,
        "type_name": type_name,
        "status": status,
        "text": f"[{type_name}] cmd=0x{cmd_id:02X} seq={seq} len={payload_len}"
        + (f" status={status}" if status is not None else "")
        + f" payload={payload.hex(' ').upper()}",
    }
