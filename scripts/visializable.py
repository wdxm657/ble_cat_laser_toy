"""
Radar prediction debug visualizer.

- Serial: text lines after 8-char log prefix, e.g. PREV,...,RAW,... / PRED,STA,... / PREDSEQ,...
- BLE: binary EVENT on Ctrl TX (cmd 0x57): prediction debug + SECTOR (0x05).
  SECTOR is **not** streamed: device sends 1 NOTIFY (sub=0x05, 13 B) only after CMD **0x57** on Ctrl RX.
  **CMD 0x58** (RADAR_TRACK_SPEED): APP writes Ctrl RX with payload u16 LE ``interval_us`` (µs)
  to set radar track gimbal step interval (same as ``StepMotor_GimbalSetSpeedUs``).
  The night-mode UI can send 0x58 from the right panel.
  Script defaults to requesting the sector on connect; use --no-ble-request-boundary to skip.
  The UI draws the annular sector boundary after receiving the single SECTOR notify.
  pip install bleak
  (Bleak 0.x uses get_services(); 1.0+ discovers on connect and exposes client.services.)

Why BLE might drop soon after connect (not caused by this script "giving up"):

1. Firmware requests conn params including supervision timeout ~4s (app.c
   bls_l2cap_requestConnParamUpdate(..., CONN_TIMEOUT_4S)). If the Windows
   radio/host misses connection events for longer than that, the link times out
   (HCI connection timeout).

2. If start_notify fails (wrong UUID, permissions), Bleak exits the client
   context and the connection ends; older code hid this with a bare except.

This script reconnects in a loop and prints errors to stderr.
"""

import argparse
import asyncio
import datetime
import math
import os
import queue
import struct
import sys
import threading
import time
import traceback
import uuid
from collections import deque
from typing import List, Optional, Set, Tuple

import serial
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch
from visualizer import commands as vc
from visualizer import protocol as cp

# Default serial (when --transport serial)
PORT = "COM3"
BAUDRATE = 115200

# Ctrl TX notify (same octet order as app_att.c CUSTOM_CTRL_TX_CHAR_UUID).
CTRL_TX_RAW_BYTES = bytes(
    [
        0x02,
        0xA0,
        0x0D,
        0x0C,
        0x0B,
        0x0A,
        0x09,
        0x08,
        0x07,
        0x06,
        0x05,
        0x04,
        0x03,
        0x02,
        0x01,
        0x00,
    ]
)
# RFC-4122 string for those bytes (used as first guess; Windows may expose reversed 128-bit order).
CTRL_TX_UUID = str(uuid.UUID(bytes=CTRL_TX_RAW_BYTES))

# Ctrl RX write (01 A0 ... per app_att.c)
CTRL_RX_RAW_BYTES = bytes(
    [
        0x01,
        0xA0,
        0x0D,
        0x0C,
        0x0B,
        0x0A,
        0x09,
        0x08,
        0x07,
        0x06,
        0x05,
        0x04,
        0x03,
        0x02,
        0x01,
        0x00,
    ]
)
CTRL_RX_UUID = str(uuid.UUID(bytes=CTRL_RX_RAW_BYTES))

# Dedicated BLE_LOG_D notify characteristic (03 A0 ... per app_att.c CUSTOM_CTRL_LOG_CHAR_UUID).
CTRL_LOG_RAW_BYTES = bytes(
    [
        0x03,
        0xA0,
        0x0D,
        0x0C,
        0x0B,
        0x0A,
        0x09,
        0x08,
        0x07,
        0x06,
        0x05,
        0x04,
        0x03,
        0x02,
        0x01,
        0x00,
    ]
)
CTRL_LOG_UUID = str(uuid.UUID(bytes=CTRL_LOG_RAW_BYTES))


def _ctrl_tx_uuid_int_candidates() -> Set[int]:
    """Same logical characteristic under different 128-bit byte orders."""
    return {
        uuid.UUID(bytes=CTRL_TX_RAW_BYTES).int,
        uuid.UUID(bytes=CTRL_TX_RAW_BYTES[::-1]).int,
    }


def _characteristic_uuid_int(char) -> Optional[int]:
    try:
        u = char.uuid
        if isinstance(u, uuid.UUID):
            return u.int
        return uuid.UUID(str(u)).int
    except Exception:
        return None


def _ctrl_rx_uuid_int_candidates() -> Set[int]:
    return {
        uuid.UUID(bytes=CTRL_RX_RAW_BYTES).int,
        uuid.UUID(bytes=CTRL_RX_RAW_BYTES[::-1]).int,
    }


def _ctrl_log_uuid_int_candidates() -> Set[int]:
    return {
        uuid.UUID(bytes=CTRL_LOG_RAW_BYTES).int,
        uuid.UUID(bytes=CTRL_LOG_RAW_BYTES[::-1]).int,
    }


def _find_ctrl_tx_characteristic(client) -> Optional[object]:
    """Return Bleak GATT characteristic for Ctrl TX, or None."""
    targets = _ctrl_tx_uuid_int_candidates()
    for svc in client.services:
        for char in svc.characteristics:
            ci = _characteristic_uuid_int(char)
            if ci is not None and ci in targets:
                return char
    return None


def _find_ctrl_rx_characteristic(client) -> Optional[object]:
    """Return Bleak GATT characteristic for Ctrl RX (write), or None."""
    targets = _ctrl_rx_uuid_int_candidates()
    for svc in client.services:
        for char in svc.characteristics:
            ci = _characteristic_uuid_int(char)
            if ci is not None and ci in targets:
                return char
    return None


def _find_ctrl_log_characteristic(client) -> Optional[object]:
    """Return Bleak GATT characteristic for dedicated Log TX, or None."""
    targets = _ctrl_log_uuid_int_candidates()
    for svc in client.services:
        for char in svc.characteristics:
            ci = _characteristic_uuid_int(char)
            if ci is not None and ci in targets:
                return char
    return None


def _dump_gatt_table(client) -> None:
    print("[BLE] GATT dump (service -> characteristics):", file=sys.stderr)
    try:
        for svc in client.services:
            su = str(svc.uuid)
            for char in svc.characteristics:
                props = ",".join(char.properties) if char.properties else ""
                print(f"  {su} -> {char.uuid} [{props}]", file=sys.stderr)
    except Exception as ex:
        print(f"  (dump failed: {ex})", file=sys.stderr)


async def _bleak_ensure_gatt_ready(client) -> None:
    """Bleak 0.x: ``await get_services()``. Bleak 1.0+: GATT on connect, use ``client.services``."""
    gs = getattr(client, "get_services", None)
    if gs is not None and callable(gs):
        out = gs()
        if asyncio.iscoroutine(out):
            await out
        return
    # Touch collection so we fail clearly if discovery did not run (per bleak docs).
    _ = client.services


# Optional keepalive: standard GAP Device Name (read-only on most peripherals)
GAP_DEVICE_NAME_UUID = "00002a00-0000-1000-8000-00805f9b34fb"

CTRL_PROTO_VERSION = cp.CTRL_PROTO_VERSION
CTRL_MSG_TYPE_CMD = cp.CTRL_MSG_TYPE_CMD
CTRL_MSG_TYPE_EVENT = cp.CTRL_MSG_TYPE_EVENT
CTRL_MSG_TYPE_RSP = cp.CTRL_MSG_TYPE_RSP
CTRL_CMD_TEXT_CHUNK = cp.CTRL_CMD_TEXT_CHUNK
CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY = cp.CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY
CTRL_CMD_RADAR_TRACK_SPEED = cp.CTRL_CMD_RADAR_TRACK_SPEED
CTRL_CMD_RADAR_PAN_OFFSET = 0x51
CTRL_CMD_FW_VERSION_GET = 0x5B
CTRL_CMD_OTA_STATUS_EVENT = 0x5C
CTRL_CMD_DEVICE_REBOOT = cp.CTRL_CMD_DEVICE_REBOOT

# ===== OTA (Telink BLE OTA) =====
# OTA Service UUID: TELINK_OTA_UUID_SERVICE
OTA_SERVICE_RAW_BYTES = bytes([
    0x12, 0x19, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00,
])
# OTA Data characteristic UUID: TELINK_SPP_DATA_OTA
OTA_DATA_RAW_BYTES = bytes([
    0x12, 0x2B, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00,
])

# OTA opcodes
CMD_OTA_VERSION = 0xFF00
CMD_OTA_START = 0xFF01
CMD_OTA_END = 0xFF02

# OTA data packet: Adr_Index(2) + Data(16) + CRC16(2) = 20 bytes
OTA_PDU_DATA_LEN = 16
OTA_PDU_TOTAL_LEN = 20

# OTA timeout default (seconds)
OTA_TIMEOUT_DEFAULT_S = 30

# Firmware size location in .bin file (offset 0x18, 4 bytes LE)
FW_SIZE_BIN_OFFSET = 0x18


def crc16_ota(data: bytes) -> int:
    """CRC-16 per Telink OTA appendix: poly 0xA001, init 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        ds = b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if (crc ^ ds) & 1 else 0)
            ds >>= 1
    return crc & 0xFFFF


def crc32_ota(data: bytes) -> int:
    """CRC-32 matching Telink OTA crc32_half_cal (half-byte table, poly 0xEDB88320).
    
    Mirrors the device-side flash_fw_check.c implementation:
    1. Each byte is split into low nibble first, high nibble second
    2. CRC = (CRC >> 4) ^ table[(CRC & 0x0F) ^ nibble]
    3. Initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF
    """
    table = [
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
    ]
    crc = 0xFFFFFFFF
    for b in data:
        # low nibble first
        n = b & 0x0F
        crc = (crc >> 4) ^ table[(crc & 0x0F) ^ n]
        # high nibble second
        n = (b >> 4) & 0x0F
        crc = (crc >> 4) ^ table[(crc & 0x0F) ^ n]
    return crc ^ 0xFFFFFFFF


def _ota_uuid_int_candidates(raw: bytes) -> Set[int]:
    return {
        uuid.UUID(bytes=raw).int,
        uuid.UUID(bytes=raw[::-1]).int,
    }


def _find_ota_characteristic(client) -> Optional[object]:
    """Return Bleak GATT characteristic for OTA data, or None."""
    targets = _ota_uuid_int_candidates(OTA_DATA_RAW_BYTES)
    for svc in client.services:
        for char in svc.characteristics:
            ci = _characteristic_uuid_int(char)
            if ci is not None and ci in targets:
                return char
    return None


def build_ota_cmd(opcode: int, payload: bytes = b"") -> bytes:
    """Build an OTA command PDU: opcode(2 bytes LE) + payload."""
    return struct.pack("<H", opcode & 0xFFFF) + payload


def build_ota_data_packet(adr_index: int, data_chunk: bytes, is_last: bool = False, fw_crc32: int = 0) -> bytes:
    """Build one OTA data PDU (20 bytes).

    Args:
        adr_index: block index = byte_offset / 16
        data_chunk: up to 16 bytes of firmware data (padded with 0xFF if short)
        is_last: if True, embed fw_crc32 in data[0:4]
        fw_crc32: CRC-32 of the entire firmware (only used when is_last=True)
    Returns:
        20 bytes: Adr_Index(2) + Data(16) + CRC16(2)
    """
    # Pad to 16 bytes with 0xFF
    if len(data_chunk) < OTA_PDU_DATA_LEN:
        data_chunk = data_chunk + b"\xFF" * (OTA_PDU_DATA_LEN - len(data_chunk))

    if is_last:
        # Last packet: data[0:4] = CRC32 of entire firmware (LE)
        data = struct.pack("<I", fw_crc32 & 0xFFFFFFFF) + data_chunk[4:]
    else:
        data = data_chunk

    pdu = struct.pack("<H", adr_index & 0xFFFF) + data  # 2 + 16 = 18 bytes
    crc = crc16_ota(pdu)
    return pdu + struct.pack("<H", crc)


def read_firmware_size(bin_path: str) -> int:
    """Read firmware size from .bin file offset 0x18 (4 bytes LE)."""
    with open(bin_path, "rb") as f:
        f.seek(FW_SIZE_BIN_OFFSET)
        raw = f.read(4)
        if len(raw) < 4:
            raise ValueError(f"Firmware file too small: {bin_path}")
        return struct.unpack("<I", raw)[0]


# x: [-1100, 1100], y: [100, 4100]
X_MIN, X_MAX = -2500, 2500
Y_MIN, Y_MAX = 100, 7000

# From RAW point, arrow length in mm (matches firmware sin/cos(motion_rad) step convention)
MOTION_ARROW_MM = 480.0

# 环形扇区默认参数 (cx, cy, inner_r_mm, outer_r_mm, angle_start_deg10, angle_end_deg10)
SECTOR_DEFAULT = (0, 0, 500, 6000, -600, 600)

# 与固件 StepMotor_ClampIntervalUs 下限一致；上位机发送前也会夹紧
TRACK_INTERVAL_US_MIN = 750
TRACK_INTERVAL_US_MAX = 20000
TRACK_INTERVAL_DEFAULT_US = 800

RADAR_BOUNDARY_ERR_TEXT = {
    0: "OK",
    1: "EDGE_TOO_SHORT(<1m)",
    2: "ORDER_INVALID",
    3: "STATE_INVALID",
    4: "INDEX_INVALID",
}


def build_ctrl_cmd_frame(cmd_id: int, seq: int, payload: bytes) -> bytes:
    return cp.build_ctrl_cmd_frame(cmd_id, seq, payload)


class RadarVisualizer:
    """Shared state; background thread reads serial or BLE notifications."""

    def __init__(
        self,
        transport: str,
        port: str = PORT,
        baudrate: int = BAUDRATE,
        ble_address: Optional[str] = None,
        ble_list_gatt: bool = False,
        ble_request_boundary: bool = True,
    ):
        self._transport = transport
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._ser = None
        self._ble_address = ble_address
        self._ble_enabled = bool(ble_address)
        self._ble_list_gatt = ble_list_gatt
        self._ble_request_boundary = ble_request_boundary
        self._ble_thread: Optional[threading.Thread] = None
        self._ble_ctrl_tx_char: Optional[object] = None
        self._ble_ctrl_rx_char: Optional[object] = None
        self._ble_tx_queue = queue.Queue()
        self._ble_tx_seq: int = 0

        self.seq_history = deque(maxlen=9)

        self.latest_prev = None
        self.latest_raw = None
        self.latest_pred_a = None
        self.latest_pred_b = None

        self.motion_dir_valid = 0
        self.motion_dir_deg10 = 0

        # 环形扇区参数 (cx, cy, inner_r, outer_r, angle_start_deg10, angle_end_deg10)
        self.sector_region: Tuple[int, int, int, int, int, int] = SECTOR_DEFAULT
        self.sector_epoch = 0

        self.log_lines = deque(maxlen=2000)
        self.ctrl_lines = deque(maxlen=2000)

        # BLE 断开连接次数统计
        self._ble_disconnect_count: int = 0
        self._ble_last_disconnect_time: Optional[str] = None
        self._ble_disconnect_need_stop_auto_power: bool = False

        # 固件版本 & OTA 状态
        self.fw_version: Optional[str] = None
        self.ota_status: int = 0          # 0=空闲 1=更新中 2=成功 3=失败
        self.ota_status_epoch: int = 0

        # BLE text chunk reassembly (EVENT 0x40). Firmware streams in-order chunks.
        self._text_rx_transfer_id: Optional[int] = None
        self._text_rx_chunk_total: int = 0
        self._text_rx_next_chunk: int = 0
        self._text_rx_buf = bytearray()
        self._ble_log_rx_buf = bytearray()

        # OTA state
        self._ota_fw_path: Optional[str] = None
        self._ota_running: bool = False
        self._ota_progress: float = 0.0  # 0.0 ~ 1.0
        self._ota_status: str = ""
        self._ota_char: Optional[object] = None
        self._ota_cmd_queue: queue.Queue = queue.Queue()
        self._ota_trigger_queue: queue.Queue = queue.Queue()  # put fw_path to trigger OTA

        if transport == "serial":
            self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

    def ota_state(self) -> tuple:
        """Return (running, progress, status)."""
        with self._lock:
            return self._ota_running, self._ota_progress, self._ota_status

    def start_ota(self, fw_path: str) -> bool:
        """Start OTA firmware upgrade in background. Returns True if started."""
        if self._transport != "ble":
            return False
        if not os.path.isfile(fw_path):
            return False
        with self._lock:
            if self._ota_running:
                return False
            self._ota_fw_path = fw_path
            self._ota_running = True
            self._ota_progress = 0.0
            self._ota_status = "OTA 初始化..."
        # Signal the BLE async loop via trigger queue
        self._ota_trigger_queue.put(fw_path)
        return True

    def _set_ota_progress(self, progress: float, status: str) -> None:
        with self._lock:
            self._ota_progress = max(0.0, min(1.0, progress))
            self._ota_status = status
            if progress >= 1.0:
                self._ota_running = False

    def _ota_abort(self, reason: str) -> None:
        with self._lock:
            self._ota_running = False
            self._ota_progress = 0.0
            self._ota_status = f"OTA 失败: {reason}"

    def send_ota_version_request(self) -> None:
        """Queue CMD_OTA_VERSION (0xFF00) to be sent via OTA characteristic."""
        if self._transport != "ble":
            return
        self._ota_cmd_queue.put(CMD_OTA_VERSION)

    def send_radar_pan_tilt_offset_deg10(self, pan: int, tilt: int) -> bool:
        """下发 CTRL_CMD_RADAR_PAN_OFFSET (0x51)，payload pan(s16 LE) + tilt(s16 LE)。仅 BLE。"""
        if self._transport != "ble":
            return False
        p = max(-3000, min(3000, int(pan)))
        t = max(-3000, min(3000, int(tilt)))
        self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
        pl = bytes([p & 0xFF, (p >> 8) & 0xFF, t & 0xFF, (t >> 8) & 0xFF])
        frame = build_ctrl_cmd_frame(CTRL_CMD_RADAR_PAN_OFFSET, self._ble_tx_seq, pl)
        self._ble_tx_queue.put(frame)
        return True

    def send_radar_track_interval_us(self, interval_us: int) -> bool:
        """下发 CTRL_CMD_RADAR_TRACK_SPEED (0x58)，payload u16 LE interval_us。仅 BLE。"""
        if self._transport != "ble":
            return False
        v = max(TRACK_INTERVAL_US_MIN, min(TRACK_INTERVAL_US_MAX, int(interval_us)))
        self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
        pl = cp.u16le(v)
        frame = build_ctrl_cmd_frame(CTRL_CMD_RADAR_TRACK_SPEED, self._ble_tx_seq, pl)
        self._ble_tx_queue.put(frame)
        return True

    def send_cmd(self, cmd_id: int, payload: bytes, note: str = "") -> bool:
        if self._transport != "ble":
            return False
        self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
        frame = build_ctrl_cmd_frame(cmd_id, self._ble_tx_seq, payload)
        self._ble_tx_queue.put(frame)
        with self._lock:
            if note:
                self.ctrl_lines.append(
                    f"[TX] cmd=0x{cmd_id:02X} seq={self._ble_tx_seq} {note}"
                )
            else:
                self.ctrl_lines.append(
                    f"[TX] cmd=0x{cmd_id:02X} seq={self._ble_tx_seq} pl={payload.hex()}"
                )
        return True

    def start(self):
        if self._transport == "serial":
            t = threading.Thread(target=self._reader_loop_serial, daemon=True)
            t.start()
        else:
            self._ble_thread = threading.Thread(target=self._ble_worker, daemon=True)
            self._ble_thread.start()

    def set_ble_target(self, address: str, enabled: bool) -> None:
        with self._lock:
            self._ble_address = (address or "").strip()
            self._ble_enabled = bool(enabled)

    def ble_target(self) -> tuple[str, bool]:
        with self._lock:
            return (self._ble_address or ""), bool(self._ble_enabled)

    def close(self):
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    def _reader_loop_serial(self):
        assert self._ser is not None
        while not self._stop.is_set():
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                with self._lock:
                    self.log_lines.append(line)
                self._parse_line(line)
            except Exception:
                time.sleep(0.01)

    def _ble_worker(self):
        try:
            asyncio.run(self._ble_async())
        except Exception:
            print("[BLE] asyncio.run failed:", file=sys.stderr)
            traceback.print_exc()

    async def _ble_async(self):
        from bleak import BleakClient

        def on_notify(_handle, data: bytearray):
            b = bytes(data)
            self._apply_ble_frame(b)

        def on_log_notify(_handle, data: bytearray):
            self._apply_ble_log_bytes(bytes(data))

        def on_disconnected(*_args):
            print("[BLE] stack reported disconnect", file=sys.stderr)
            with self._lock:
                self._ble_disconnect_count += 1
                self._ble_last_disconnect_time = datetime.datetime.now().strftime("%H:%M:%S")
                self._ble_disconnect_need_stop_auto_power = True

        # Reconnect loop: peripheral or host may drop link (supervision ~4s on this firmware).
        while not self._stop.is_set():
            address, enabled = self.ble_target()
            if (not enabled) or (not address):
                await asyncio.sleep(0.2)
                continue
            try:
                async with BleakClient(
                    address,
                    disconnected_callback=on_disconnected,
                ) as client:
                    print("[BLE] connected", file=sys.stderr)
                    await _bleak_ensure_gatt_ready(client)
                    if self._ble_list_gatt:
                        _dump_gatt_table(client)
                    ctrl_tx = _find_ctrl_tx_characteristic(client)
                    if ctrl_tx is None:
                        _dump_gatt_table(client)
                        raise RuntimeError(
                            "Ctrl TX characteristic not found (custom notify). "
                            "Confirm firmware is ble_cat_laser_toy and check GATT dump above."
                        )
                    self._ble_ctrl_tx_char = ctrl_tx
                    rx_ch = _find_ctrl_rx_characteristic(client)
                    self._ble_ctrl_rx_char = rx_ch
                    if rx_ch is None:
                        print(
                            "[BLE] Ctrl RX not found; track speed writes disabled",
                            file=sys.stderr,
                        )
                    log_tx = _find_ctrl_log_characteristic(client)
                    if log_tx is None:
                        print(
                            "[BLE] Log TX characteristic not found; BLE_LOG_D display disabled",
                            file=sys.stderr,
                        )
                    print(f"[BLE] notify on {ctrl_tx.uuid}", file=sys.stderr)
                    await client.start_notify(ctrl_tx, on_notify)
                    if log_tx is not None:
                        print(f"[BLE] log notify on {log_tx.uuid}", file=sys.stderr)
                        await client.start_notify(log_tx, on_log_notify)

                    # Auto time sync on every successful connect.
                    # This makes firmware print app_radar_set_time_from_epoch() logs immediately after connect.
                    if rx_ch is not None:
                        try:
                            epoch_sec = int(time.time())
                            tz_q15 = 0
                            try:
                                now = datetime.datetime.now(
                                    datetime.timezone.utc
                                ).astimezone()
                                off = now.utcoffset()
                                off_sec = int(off.total_seconds()) if off else 0
                                tz_q15 = int(round(off_sec / 900.0))
                                tz_q15 = max(-128, min(127, tz_q15))
                            except Exception:
                                pass

                            self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
                            payload = vc.cmd_time_set(epoch_sec, tz_q15)[1]
                            frame = build_ctrl_cmd_frame(
                                cp.CTRL_CMD_TIME_SET, self._ble_tx_seq, payload
                            )
                            try:
                                await client.write_gatt_char(
                                    rx_ch, frame, response=True
                                )
                            except Exception:
                                await client.write_gatt_char(
                                    rx_ch, frame, response=False
                                )
                            with self._lock:
                                self.ctrl_lines.append(
                                    f"[TX] cmd=0x32 seq={self._ble_tx_seq} TIME_SET epoch={epoch_sec} tz_q15={tz_q15}"
                                )
                        except Exception as ex:
                            print("[BLE] TIME_SET write failed:", ex, file=sys.stderr)
                    if self._ble_request_boundary:

                        async def _write_get_boundary(rx_char) -> None:
                            req = bytes(
                                [
                                    CTRL_PROTO_VERSION,
                                    CTRL_MSG_TYPE_CMD,
                                    CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY,
                                    0,
                                    0,
                                    0,
                                ]
                            )
                            try:
                                await client.write_gatt_char(
                                    rx_char, req, response=True
                                )
                            except Exception:
                                await client.write_gatt_char(
                                    rx_char, req, response=False
                                )

                        await asyncio.sleep(0.25)
                        try:
                            if rx_ch is None:
                                print(
                                    "[BLE] Ctrl RX not found; cannot request boundary quad",
                                    file=sys.stderr,
                                )
                            else:
                                await _write_get_boundary(rx_ch)
                                print("[BLE] sent GET_BOUNDARY (0x57)", file=sys.stderr)
                                await asyncio.sleep(0.3)

                                # 请求固件版本
                                self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
                                req_ver = bytes([
                                    CTRL_PROTO_VERSION, CTRL_MSG_TYPE_CMD,
                                    CTRL_CMD_FW_VERSION_GET, self._ble_tx_seq,
                                    0, 0
                                ])
                                try:
                                    await client.write_gatt_char(rx_ch, req_ver, response=True)
                                except Exception:
                                    await client.write_gatt_char(rx_ch, req_ver, response=False)
                                print("[BLE] sent FW_VERSION_GET (0x5B)", file=sys.stderr)
                                await asyncio.sleep(0.3)

                                with self._lock:
                                    need_retry = self.sector_epoch == 0
                                if need_retry:
                                    print(
                                        "[BLE] no BOUNDARY_PT yet; retry GET_BOUNDARY",
                                        file=sys.stderr,
                                    )
                                    await _write_get_boundary(rx_ch)
                        except Exception as ex:
                            print(
                                "[BLE] GET_BOUNDARY write failed:",
                                ex,
                                file=sys.stderr,
                            )
                    # Process any queued OTA trigger (from keepalive loop)
                    keepalive = 0
                    while not self._stop.is_set():
                        a2, e2 = self.ble_target()
                        if (not e2) or (not a2):
                            print(
                                "[BLE] disabled by UI, disconnecting", file=sys.stderr
                            )
                            break
                        if not client.is_connected:
                            print(
                                "[BLE] is_connected=False, will reconnect",
                                file=sys.stderr,
                            )
                            break
                        try:
                            while True:
                                pkt = self._ble_tx_queue.get_nowait()
                                rxw = self._ble_ctrl_rx_char
                                if rxw is not None:
                                    try:
                                        await client.write_gatt_char(
                                            rxw, pkt, response=False
                                        )
                                    except Exception as ex:
                                        print(
                                            f"[BLE] write queue failed: {ex}",
                                            file=sys.stderr,
                                        )
                        except queue.Empty:
                            pass
                        # OTA command queue (e.g. CMD_OTA_VERSION)
                        try:
                            ota_opcode = self._ota_cmd_queue.get_nowait()
                            ota_ch = _find_ota_characteristic(client)
                            if ota_ch is not None:
                                pkt = build_ota_cmd(ota_opcode)
                                await client.write_gatt_char(
                                    ota_ch, pkt, response=False
                                )
                                with self._lock:
                                    self.ctrl_lines.append(
                                        f"[OTA][TX] cmd=0x{ota_opcode:04X}"
                                    )
                        except queue.Empty:
                            pass
                        # OTA firmware upgrade trigger (from start_ota queue)
                        try:
                            ota_fw_now = self._ota_trigger_queue.get_nowait()
                            print(f"[OTA] trigger received: {ota_fw_now}", file=sys.stderr)
                            await self._ota_async_send(client, ota_fw_now)
                        except queue.Empty:
                            pass
                        keepalive += 1
                        # Light GATT read ~every 2s to nudge some Windows stacks / central scheduling.
                        if keepalive >= 40:
                            keepalive = 0
                            try:
                                await client.read_gatt_char(GAP_DEVICE_NAME_UUID)
                            except Exception:
                                pass
                        await asyncio.sleep(0.05)
                    try:
                        await client.stop_notify(ctrl_tx)
                    except Exception:
                        pass
                    if log_tx is not None:
                        try:
                            await client.stop_notify(log_tx)
                        except Exception:
                            pass
                    self._ble_ctrl_tx_char = None
                    self._ble_ctrl_rx_char = None
            except Exception as ex:
                if self._stop.is_set():
                    break
                print("[BLE] session error (reconnecting):", ex, file=sys.stderr)
                with self._lock:
                    self._ble_disconnect_count += 1
                    self._ble_last_disconnect_time = datetime.datetime.now().strftime("%H:%M:%S")
                    self._ble_disconnect_need_stop_auto_power = True
                traceback.print_exc()
                await asyncio.sleep(1.5)

    async def _ota_async_send(self, client, fw_path: str) -> None:
        """Execute full OTA flow: find OTA char, send START, data, END.

        Must be called from within an active BleakClient context.
        """
        def _ota_log(msg: str) -> None:
            with self._lock:
                self.ctrl_lines.append(f"[OTA] {msg}")
                self.log_lines.append(f"[OTA] {msg}")

        _ota_log("开始 OTA 流程")

        # 1. Find OTA characteristic
        ota_char = _find_ota_characteristic(client)
        if ota_char is None:
            _ota_log("OTA characteristic 未找到!")
            self._ota_abort("OTA characteristic 未找到")
            return

        _ota_log(f"OTA characteristic 已找到: {ota_char.uuid}, handle={ota_char.handle}")

        # Read firmware file
        fw_size = 0
        with open(fw_path, "rb") as f:
            fw_data = f.read()
        if len(fw_data) < 4:
            _ota_log("固件文件无效（太小）")
            self._ota_abort("固件文件无效（太小）")
            return

        # 2. Read firmware size from bin offset 0x18
        try:
            fw_size = read_firmware_size(fw_path)
        except Exception as ex:
            _ota_log(f"读取固件大小失败: {ex}")
            self._ota_abort(f"读取固件大小失败: {ex}")
            return

        if fw_size > len(fw_data):
            fw_size = len(fw_data)

        # Firmware_size at bin offset 0x18 includes the 4-byte CRC32 trailer.
        # tl_check_fw2.exe has now correctly appended CRC32 in the .bin file.
        # Standard OTA protocol per doc:
        #   - total_data_packets = ceil(fw_size / 16)
        #   - All data packets from fw_data (includes CRC32 at end)
        #   - Last packet embeds CRC32 at data[0:4] (is_last=True)
        #   - No separate CRC packet
        #   - max_adr_index = total_data_packets - 1
        code_size = fw_size - 4
        if code_size < 1:
            _ota_log("固件数据为空")
            self._ota_abort("固件数据为空")
            return

        total_data_packets = (fw_size + OTA_PDU_DATA_LEN - 1) // OTA_PDU_DATA_LEN

        # CRC32 from binary — computed by tl_check_fw2.exe, matching library.
        fw_crc32 = struct.unpack('<I', fw_data[fw_size - 4:fw_size])[0]
        if total_data_packets < 1:
            _ota_log("固件数据为空")
            self._ota_abort("固件数据为空")
            return

        # Dump 5 offsets (known to differ between OLD and NEW firmware)
        _dump_offsets = [0x00018, 0x00170, 0x06F78, 0x0ADC4, 0x1F0D0]
        _ota_log(f"  PC bin dump @5 offsets:")
        for _off in _dump_offsets:
            if _off + 4 <= len(fw_data):
                _val = struct.unpack('<I', fw_data[_off:_off+4])[0]
                _ota_log(f"    [0x{_off:05X}]=0x{_val:08X}")
        # Also read old firmware if available
        _old_path = fw_path.replace('_ABC', '')
        try:
            if os.path.isfile(_old_path) and _old_path != fw_path:
                with open(_old_path, 'rb') as _f:
                    _old = _f.read()
                _ota_log(f"  OLD bin (device) dump:")
                for _off in _dump_offsets:
                    if _off + 4 <= len(_old):
                        _val = struct.unpack('<I', _old[_off:_off+4])[0]
                        _ota_log(f"    [0x{_off:05X}]=0x{_val:08X}")
        except Exception:
            pass
        _ota_log(f"固件大小={fw_size} 字节, 代码数据={code_size} 字节, 数据包数={total_data_packets}, CRC32=0x{fw_crc32:08X}")

        self._set_ota_progress(0.0, f"OTA 开始: {total_data_packets} 数据包, CRC32=0x{fw_crc32:08X}")

        # 4. Send OTA_START command (opcode 0xFF01, no payload)
        start_cmd = build_ota_cmd(CMD_OTA_START)
        _ota_log("发送 OTA_START (0xFF01)")
        try:
            await client.write_gatt_char(ota_char, start_cmd, response=False)
        except Exception as ex:
            _ota_log(f"OTA_START 发送失败: {ex}")
            self._ota_abort(f"OTA_START 发送失败: {ex}")
            return
        _ota_log("OTA_START 已发送")

        # Wait for device to enter OTA mode and finish flash erase setup.
        # The device may stall during erase, so give it enough time to avoid supervision timeout.
        _ota_log("等待设备 OTA 准备...")
        await asyncio.sleep(2.0)

        # Check connection is still alive after the wait
        if not client.is_connected:
            _ota_log("设备在 OTA 准备阶段断连!")
            self._ota_abort("设备在 OTA 准备阶段断连")
            return
        _ota_log("连接正常，开始发送数据")

        # 5. Send all OTA data packets.
        # Use Write Command (response=False) and a short delay between packets.
        # The delay prevents Windows BLE TX buffer overflow while keeping high throughput.
        _ota_log(f"开始发送 {total_data_packets} 个数据包 (每包间隔 ~2ms)")
        sent = 0
        last_report = 0
        for i in range(total_data_packets):
            if self._stop.is_set():
                self._ota_abort("用户中断")
                return

            offset = i * OTA_PDU_DATA_LEN
            chunk = fw_data[offset : offset + OTA_PDU_DATA_LEN]
            # Last packet: embed CRC32 at data[0:4] per OTA doc.
            is_last = (i == total_data_packets - 1)
            pkt = build_ota_data_packet(i, chunk, is_last=is_last, fw_crc32=fw_crc32)
            try:
                await client.write_gatt_char(ota_char, pkt, response=False)
            except Exception as ex:
                _ota_log(f"数据包 {i} 发送失败: {ex}")
                self._ota_abort(f"数据包 {i} 发送失败: {ex}")
                return

            sent += 1

            # Report progress at 1% intervals for smoother bar
            progress = sent / total_data_packets
            progress_pct = int(progress * 100)
            if progress_pct >= last_report + 1 or i == 0:
                last_report = progress_pct
                self._set_ota_progress(
                    progress, f"OTA 发送中... {sent}/{total_data_packets} ({progress * 100:.1f}%)"
                )

            await asyncio.sleep(0.002)

        _ota_log(f"数据包发送完毕: {total_data_packets} 包, 等待 TX buffer 排空...")
        await asyncio.sleep(1.0)

        if not client.is_connected:
            _ota_log("连接已断开")
            self._ota_abort("连接已断开")
            return
        _ota_log("TX buffer 已排空")

        # adr_index_max = last data packet index (per OTA doc: "最大的adr_index值")
        max_adr_index = total_data_packets - 1
        await asyncio.sleep(0.5)
        self._set_ota_progress(0.95, "OTA 数据发送完毕，发送 OTA_END...")

        # 7. Check connection before OTA_END
        if not client.is_connected:
            _ota_log("连接已断开，无法发送 OTA_END")
            self._ota_abort("连接已断开，OTA_END 未发送")
            return

        # 8. Send OTA_END command, retry once if needed
        # OTA_END payload: adr_index_max(2 LE) + ~adr_index_max(2 LE)
        adr_max_xor = (~max_adr_index) & 0xFFFF
        end_payload = struct.pack("<HH", max_adr_index & 0xFFFF, adr_max_xor)
        end_cmd = build_ota_cmd(CMD_OTA_END, end_payload)
        _ota_log(f"发送 OTA_END (0xFF02): adr_index_max=0x{max_adr_index:04X}, xor=0x{adr_max_xor:04X}")
        for retry in range(2):
            try:
                await client.write_gatt_char(ota_char, end_cmd, response=False)
                _ota_log(f"OTA_END 已发送 (第{retry + 1}次)")
                break
            except Exception as ex:
                if retry == 0:
                    _ota_log(f"OTA_END 发送失败，重试: {ex}")
                    await asyncio.sleep(0.5)
                else:
                    _ota_log(f"OTA_END 发送失败: {ex}")
                    self._ota_abort(f"OTA_END 发送失败: {ex}")
                    return

        # 9. Final wait for device to process OTA_END and reboot
        await asyncio.sleep(0.5)

        self._set_ota_progress(
            1.0,
            f"OTA 完成! 共 {total_data_packets} 数据包 + END, 设备将重启.",
        )
        _ota_log("OTA 流程完成，设备应重启")

    def _apply_ble_log_bytes(self, data: bytes) -> None:
        if not data:
            return

        complete_lines = []
        with self._lock:
            self._ble_log_rx_buf += data
            while True:
                try:
                    nl = self._ble_log_rx_buf.index(0x0A)
                except ValueError:
                    break
                raw_line = bytes(self._ble_log_rx_buf[:nl])
                del self._ble_log_rx_buf[: nl + 1]
                raw_line = raw_line.rstrip(b"\r")
                if not raw_line:
                    continue
                line = raw_line.decode("utf-8", errors="replace")
                complete_lines.append(line)
                self.log_lines.append(line)

            # Avoid holding an unterminated partial line forever if firmware sends no newline.
            if len(self._ble_log_rx_buf) > 512:
                raw_line = bytes(self._ble_log_rx_buf)
                self._ble_log_rx_buf.clear()
                line = raw_line.decode("utf-8", errors="replace")
                complete_lines.append(line)
                self.log_lines.append(line)

        for line in complete_lines:
            self._parse_line(line)

    def _apply_ble_frame(self, data: bytes) -> None:
        # 打印16进制数据
        fr = cp.parse_ctrl_frame(data)
        if fr is None:
            return
        if fr.version != CTRL_PROTO_VERSION:
            return
        cmd_id = fr.cmd_id
        payload = fr.payload

        if fr.msg_type == CTRL_MSG_TYPE_EVENT and cmd_id == CTRL_CMD_TEXT_CHUNK:
            if not payload:
                return
            # payload: [0]=transferId, [1]=chunkIndex, [2]=chunkTotal, [3]=dataLen, [4..]=data
            if len(payload) < 4:
                return
            transfer_id = int(payload[0])
            chunk_idx = int(payload[1])
            chunk_total = int(payload[2])
            data_len = int(payload[3])
            if data_len < 0 or (4 + data_len) > len(payload):
                return
            chunk_data = payload[4 : 4 + data_len]

            complete_lines = []
            parse_hex_line = None
            with self._lock:
                # Start (or restart) on chunk 0 or transfer id change.
                if (
                    chunk_idx == 0
                    or self._text_rx_transfer_id is None
                    or self._text_rx_transfer_id != transfer_id
                ):
                    self._text_rx_transfer_id = transfer_id
                    self._text_rx_chunk_total = max(1, chunk_total)
                    self._text_rx_next_chunk = 0
                    self._text_rx_buf = bytearray()

                # Enforce in-order assembly (firmware sends sequentially).
                if chunk_idx != self._text_rx_next_chunk:
                    self._text_rx_transfer_id = None
                    self._text_rx_chunk_total = 0
                    self._text_rx_next_chunk = 0
                    self._text_rx_buf = bytearray()
                    return

                self._text_rx_buf += chunk_data
                self._text_rx_next_chunk += 1

                if self._text_rx_next_chunk >= self._text_rx_chunk_total:
                    b = bytes(self._text_rx_buf)
                    self._text_rx_transfer_id = None
                    self._text_rx_chunk_total = 0
                    self._text_rx_next_chunk = 0
                    self._text_rx_buf = bytearray()
                    try:
                        s = b.decode("utf-8", errors="replace")
                        complete_lines = s.splitlines()
                        for line in complete_lines:
                            self.log_lines.append(line)
                    except Exception:
                        parse_hex_line = "[BLE][TEXT_CHUNK] " + b.hex()
                        self.log_lines.append(parse_hex_line)

            for line in complete_lines:
                self._parse_line(line)
            if parse_hex_line is not None:
                self._parse_line(parse_hex_line)
            return

        # Radar prediction debug / boundary quad (binary event channel, cmd=0x57).
        if (
            fr.msg_type == CTRL_MSG_TYPE_EVENT
            and cmd_id == cp.CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY
        ):
            if not payload:
                return
            sub = int(payload[0])

            def _s16le(lo: int, hi: int) -> int:
                v = (int(lo) & 0xFF) | ((int(hi) & 0xFF) << 8)
                return v - 0x10000 if v >= 0x8000 else v

            # 0x01: prev/raw + motion
            if sub == 0x01 and len(payload) >= 12:
                prev_x = _s16le(payload[1], payload[2])
                prev_y = _s16le(payload[3], payload[4])
                raw_x = _s16le(payload[5], payload[6])
                raw_y = _s16le(payload[7], payload[8])
                m_valid = int(payload[9])
                m_deg10 = _s16le(payload[10], payload[11])
                self._apply_prev_raw(prev_x, prev_y, raw_x, raw_y, m_valid, m_deg10)
                return

            # 0x02: pred sta (A,B)
            if sub == 0x02 and len(payload) >= 9:
                ax = _s16le(payload[1], payload[2])
                ay = _s16le(payload[3], payload[4])
                bx = _s16le(payload[5], payload[6])
                by = _s16le(payload[7], payload[8])
                self._apply_pred_sta(ax, ay, bx, by)
                return

            # 0x03: pred seq
            if sub == 0x03 and len(payload) >= 6:
                idx = int(payload[1])
                x = _s16le(payload[2], payload[3])
                y = _s16le(payload[4], payload[5])
                self._apply_predseq(idx, x, y)
                return

            # 0x05: sector region (single notify with all 6 params)
            if sub == 0x05 and len(payload) >= 13:
                cx = _s16le(payload[1], payload[2])
                cy = _s16le(payload[3], payload[4])
                ri = int(payload[5]) | (int(payload[6]) << 8)
                ro = int(payload[7]) | (int(payload[8]) << 8)
                a_start = _s16le(payload[9], payload[10])
                a_end = _s16le(payload[11], payload[12])
                self._apply_sector_region(cx, cy, ri, ro, a_start, a_end)
                return

        # OTA 状态事件 (cmd=0x5C)
        if (
            fr.msg_type == CTRL_MSG_TYPE_EVENT
            and cmd_id == CTRL_CMD_OTA_STATUS_EVENT
        ):
            if len(payload) >= 1:
                status = int(payload[0])
                with self._lock:
                    self.ota_status = status
                    self.ota_status_epoch += 1
                    self.ctrl_lines.append(self._decode_ctrl_line(fr))
            return

        # 固件版本响应 (cmd=0x5B)
        if (
            fr.msg_type == CTRL_MSG_TYPE_RSP
            and cmd_id == CTRL_CMD_FW_VERSION_GET
        ):
            if len(payload) >= 3:
                pat = int(payload[0])
                mid = int(payload[1])
                maj = int(payload[2])
                ver_str = f"{maj}.{mid}.{pat}"
                with self._lock:
                    self.fw_version = ver_str
                    self.ctrl_lines.append(self._decode_ctrl_line(fr))
            return

        # 逗宠记录 EVENT (cmd=0x33): 记录数据
        if (
            fr.msg_type == CTRL_MSG_TYPE_EVENT
            and cmd_id == cp.CTRL_CMD_PLAY_RECORD_GET
        ):
            self._apply_play_record_event(payload)
            with self._lock:
                self.ctrl_lines.append(self._decode_ctrl_line(fr))
            return

        with self._lock:
            self.ctrl_lines.append(self._decode_ctrl_line(fr))

    def _decode_ctrl_line(self, fr: cp.CtrlFrame) -> str:
        pld = fr.payload
        if fr.msg_type == CTRL_MSG_TYPE_RSP:
            st = pld[0] if len(pld) >= 1 else -1
            if fr.cmd_id == cp.CTRL_CMD_POWER_CTRL and len(pld) >= 3:
                return f"[RSP][0x12] status={st} on_effective={pld[1]} reason={pld[2]}"
            if fr.cmd_id == cp.CTRL_CMD_POWER_CTRL and len(pld) >= 2:
                return f"[RSP][0x12] status={st} on={pld[1]}"
            if fr.cmd_id == cp.CTRL_CMD_MOTOR_DIR_CTRL and len(pld) >= 3:
                return f"[RSP][0x22] status={st} dir={pld[1]} op={pld[2]}"
            if fr.cmd_id == cp.CTRL_CMD_RADAR_RESET_FLASH_CONFIG:
                return f"[RSP][0x56] status={st}"
            if fr.cmd_id == cp.CTRL_CMD_RADAR_CONFIG_SET_HEIGHT:
                return f"[RSP][0x50] status={st} (new config: height cached)"
            if fr.cmd_id == cp.CTRL_CMD_DEVICE_REBOOT:
                return f"[RSP][0x5A] status={st} (rebooting)"
            if fr.cmd_id == cp.CTRL_CMD_PLAY_RECORD_DELETE and len(pld) >= 2:
                remaining = pld[1]
                return f"[RSP][0x35] PLAY_RECORD_DELETE status={st} remaining={remaining}"
            if fr.cmd_id == cp.CTRL_CMD_PLAY_RECORD_GET and len(pld) >= 2:
                remaining = pld[1]
                if remaining == 0:
                    return f"[RSP][0x33] status={st} remaining={remaining} (ACK已废弃, 使用0x35删除代替)"
                else:
                    return f"[RSP][0x33] status={st} remaining={remaining} (ACK已废弃, 使用0x35删除代替)"
            return f"[RSP][0x{fr.cmd_id:02X}] status={st} pl={pld.hex()}"

        if fr.msg_type == CTRL_MSG_TYPE_EVENT and fr.cmd_id == cp.CTRL_CMD_PLAY_RECORD_GET:
            if len(pld) >= 14:
                rid = pld[1]
                total = pld[2]
                idx = pld[3]
                return f"[EVT][0x33] ID={rid} 记录 {idx + 1}/{total} payload={pld.hex()}"
            return f"[EVT][0x33] pl={pld.hex()}"

        # 狩猎游戏命令 RSP 解码
        if fr.msg_type == CTRL_MSG_TYPE_RSP:
            st = pld[0] if len(pld) >= 1 else -1
            if fr.cmd_id == cp.CTRL_CMD_HUNT_SETTINGS_ENTER:
                return f"[RSP][0x60] HUNT_SETTINGS_ENTER status={st}"
            if fr.cmd_id == cp.CTRL_CMD_HUNT_SETTINGS_EXIT:
                return f"[RSP][0x61] HUNT_SETTINGS_EXIT status={st}"
            if fr.cmd_id == cp.CTRL_CMD_HUNT_PREY_RANDOM:
                start = pld[1] if len(pld) >= 2 else 0
                return f"[RSP][0x62] HUNT_PREY_RANDOM status={st} start={start}"
            if fr.cmd_id == cp.CTRL_CMD_HUNT_SETTINGS_SET and len(pld) >= 5:
                dur_s = pld[1] | (pld[2] << 8)
                cnt = pld[3]
                slp = pld[4]
                return f"[RSP][0x63] HUNT_SETTINGS_SET status={st} duration={dur_s}s count={cnt} sleep={slp}min"
            if fr.cmd_id == cp.CTRL_CMD_HUNT_SETTINGS_GET and len(pld) >= 5:
                dur_s = pld[1] | (pld[2] << 8)
                cnt = pld[3]
                slp = pld[4]
                return f"[RSP][0x64] HUNT_SETTINGS_GET status={st} duration={dur_s}s count={cnt} sleep={slp}min"
        if (
            fr.msg_type == CTRL_MSG_TYPE_EVENT
            and fr.cmd_id == cp.CTRL_CMD_MOTOR_DIR_CTRL
        ):
            if len(pld) >= 2 and pld[0] == 0x01:
                return f"[EVT][0x22] reached dir={pld[1]}"
            if len(pld) >= 4 and pld[0] == 0x02:
                return f"[EVT][0x22] boundary_guard dir={pld[1]} point={pld[2]} limit={pld[3]}"
            if len(pld) >= 4 and pld[0] == 0x03:
                tilt = pld[2] | (pld[3] << 8)
                if tilt >= 0x8000:
                    tilt -= 0x10000
                return f"[EVT][0x22] height_limit dir={pld[1]} tilt_deg10={tilt}"
        return f"[RX] type=0x{fr.msg_type:02X} cmd=0x{fr.cmd_id:02X} seq={fr.seq} pl={pld.hex()}"

    def _apply_sector_region(self, cx: int, cy: int, ri: int, ro: int,
                              a_start_deg10: int, a_end_deg10: int) -> None:
        with self._lock:
            self.sector_region = (cx, cy, ri, ro, a_start_deg10, a_end_deg10)
            self.sector_epoch += 1

    def _apply_prev_raw(
        self,
        prev_x: int,
        prev_y: int,
        raw_x: int,
        raw_y: int,
        motion_valid: int = 0,
        motion_deg10: int = 0,
    ) -> None:
        # High-rate stream: only refresh prev/raw. Do not clear STA / PREDSEQ (sticky for debug).
        with self._lock:
            self.latest_prev = (prev_x, prev_y)
            self.latest_raw = (raw_x, raw_y, None)
            self.motion_dir_valid = 1 if motion_valid else 0
            self.motion_dir_deg10 = int(motion_deg10)

    def _apply_pred_sta(self, ax: int, ay: int, bx: int, by: int) -> None:
        # Hold until next STA; do not clear seq trail (independent channel).
        with self._lock:
            self.latest_pred_a = (ax, ay)
            self.latest_pred_b = (bx, by)

    def _apply_predseq(self, idx: int, x: int, y: int) -> None:
        # Hold STA points while appending seq; new sequence only when idx==1.
        with self._lock:
            if idx == 1:
                self.seq_history.clear()
            self.seq_history.append((x, y))

    # 逗宠记录缓存（用于 UI 显示）
    _play_record_count: int = 0
    _play_record_index: int = 0
    _play_record_info: str = ""
    _play_records: list = []  # 存储所有已收到记录的列表 [(id, start_str, end_str, motion, speed, result_str)]

    def _apply_play_record_event(self, payload: bytes) -> None:
        """解析逗宠记录 EVENT payload (13 字节, 含狩猎结果)。

        BLE 20 字节限制 (CTRL_TX_MAX_LEN): 6 字节头 + payload ≤ 20。
        end_sec 用 duration_sec (u16) 替代以压缩 payload：duration_sec = end_sec - start_sec。
        APP 侧恢复：end_sec = start_sec + duration_sec。
        payload[0] 为 record_id（取代原 status 字节）。
        """
        if len(payload) < 14:
            return
        record_id = int(payload[1])  # 记录ID (payload[0]=status)
        total = int(payload[2])
        index = int(payload[3])
        start_sec = (
            int(payload[4])
            | (int(payload[5]) << 8)
            | (int(payload[6]) << 16)
            | (int(payload[7]) << 24)
        )
        duration_sec = int(payload[8]) | (int(payload[9]) << 8)  # u16 LE
        end_sec = start_sec + duration_sec
        motion_sec = int(payload[10]) | (int(payload[11]) << 8)
        avg_speed = int(payload[12])
        result = int(payload[13])

        RESULT_TEXT = {0: "未完成", 1: "完成", 2: "捕猎成功"}

        def _fmt_ts(epoch: int) -> str:
            if epoch == 0:
                return "--"
            try:
                return datetime.datetime.fromtimestamp(
                    epoch, tz=datetime.timezone.utc
                ).strftime("%m-%d %H:%M:%S")
            except Exception:
                return str(epoch)

        start_str = _fmt_ts(start_sec)
        end_str = _fmt_ts(end_sec)
        result_str = RESULT_TEXT.get(result, f"未知({result})")

        with self._lock:
            self._play_record_count = total
            self._play_record_index = index
            # 存储记录到列表（替换同ID或追加）
            found = False
            for i, rec in enumerate(self._play_records):
                if rec[0] == record_id:
                    self._play_records[i] = (record_id, start_str, end_str, motion_sec, avg_speed, result_str)
                    found = True
                    break
            if not found:
                self._play_records.append((record_id, start_str, end_str, motion_sec, avg_speed, result_str))
            # 更新当前显示信息
            self._play_record_info = (
                f"ID={record_id} 记录 {index + 1}/{total}: {start_str} ~ {end_str}  "
                f"运动 {motion_sec}s  速度 {avg_speed}cm/s  "
                f"结果: {result_str}"
            )

        # 不再自动 ACK：删除命令 (0x35) 同时充当 ACK，由用户手动触发删除

    def _parse_line(self, line: str):
        s = line.strip()
        if not s:
            return
        if s.startswith("[") and "]" in s:
            s = s.split("]", 1)[1].strip()

        parts = [p.strip() for p in s.split(",")]
        if len(parts) < 2:
            return

        try:
            if parts[0] == "PREV" and len(parts) >= 6 and parts[3] == "RAW":
                m_valid = 0
                m_deg10 = 0
                if len(parts) >= 9 and parts[6] == "M":
                    m_valid = int(parts[7])
                    m_deg10 = int(parts[8])
                self._apply_prev_raw(
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[4]),
                    int(parts[5]),
                    m_valid,
                    m_deg10,
                )
            elif parts[0] == "PRED" and parts[1] == "STA" and len(parts) >= 6:
                self._apply_pred_sta(
                    int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])
                )
            elif parts[0] == "PREDSEQ" and len(parts) >= 4:
                self._apply_predseq(int(parts[1]), int(parts[2]), int(parts[3]))
            elif parts[0] == "SECTOR" and len(parts) >= 7:
                self._apply_sector_region(
                    int(parts[1]), int(parts[2]), int(parts[3]),
                    int(parts[4]), int(parts[5]), int(parts[6]))
        except ValueError:
            return


NIGHT_STYLESHEET = """
QMainWindow, QWidget { background-color: #1e1e2e; color: #cdd6f4; }
QPlainTextEdit {
  background-color: #181825; color: #cdd6f4; border: 1px solid #45475a;
  border-radius: 6px; padding: 8px; font-family: Consolas, "Courier New", monospace;
  font-size: 12px;
}
QGroupBox {
  font-weight: bold; border: 1px solid #45475a; border-radius: 8px; margin-top: 12px; padding: 12px;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #89b4fa; }
QSpinBox, QPushButton {
  background: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 6px 12px;
}
QPushButton:hover { background: #45475a; }
QPushButton:disabled { color: #6c7086; background: #313244; }
QLabel { color: #bac2de; }
"""


class CtrlServiceWindow(QtWidgets.QMainWindow):
    """控制服务面板 - 独立窗口，包含电源/时间/电机/狩猎等控制命令"""

    def __init__(self, vis: RadarVisualizer, parent=None):
        super().__init__(parent)
        self.vis = vis
        self.setWindowTitle("控制服务面板")
        self.setMinimumWidth(500)
        self.setStyleSheet(NIGHT_STYLESHEET)
        self.setAttribute(Qt.WA_DeleteOnClose, False)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(8)

        # Build the control panel grid
        self._build_panel(root)

        # Log text area
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(100)
        self.log_text.setPlaceholderText("操作日志")
        self.log_text.document().setDefaultFont(
            QtGui.QFont("Consolas", 10)
            if sys.platform == "win32"
            else QtGui.QFont("monospace", 10)
        )
        root.addWidget(self.log_text, 1)

        # Auto power toggle timer
        self._auto_power_timer = QtCore.QTimer(self)
        self._auto_power_interval = 35
        self._auto_power_timer.setInterval(self._auto_power_interval * 1000)
        self._auto_power_timer.timeout.connect(self._on_auto_power_tick)
        self._auto_power_on_phase = True

        # Timer for periodic UI updates (play record info + disconnect check)
        self._svc_update_timer = QtCore.QTimer(self)
        self._svc_update_timer.setInterval(200)
        self._svc_update_timer.timeout.connect(self._on_svc_update_tick)
        self._svc_update_timer.start()

    def log(self, msg: str) -> None:
        self.log_text.appendPlainText(msg)

    def _send(self, cmd_id: int, payload: bytes, desc: str) -> None:
        ok = self.vis.send_cmd(cmd_id, payload, desc)
        if not ok:
            self.log("下发失败（非 BLE 或未连接）")

    def _build_panel(self, root: QtWidgets.QVBoxLayout) -> None:
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(6)
        g.setVerticalSpacing(6)
        root.addLayout(g)

        # Power
        def _power_on():
            self._stop_auto_power_if_running()
            self._send(*vc.cmd_power_ctrl(1))
        def _power_off():
            self._stop_auto_power_if_running()
            self._send(*vc.cmd_power_ctrl(0))
        b_power_on = QtWidgets.QPushButton("PowerOn")
        b_power_on.clicked.connect(_power_on)
        b_power_off = QtWidgets.QPushButton("PowerOff")
        b_power_off.clicked.connect(_power_off)
        self._auto_power_btn = QtWidgets.QPushButton("AutoToggle(30s)")
        self._auto_power_btn.setCheckable(True)
        self._auto_power_btn.clicked.connect(self._on_auto_power_toggle)
        g.addWidget(QtWidgets.QLabel("电源 (0x12)"), 0, 0)
        g.addWidget(b_power_on, 0, 1)
        g.addWidget(b_power_off, 0, 2)
        g.addWidget(self._auto_power_btn, 0, 3)

        # Time set
        g.addWidget(QtWidgets.QLabel("时间同步 (0x32)"), 1, 0)
        self.time_use_local_tz = QtWidgets.QCheckBox("本机时区")
        self.time_use_local_tz.setChecked(True)
        g.addWidget(self.time_use_local_tz, 1, 1)
        self.time_tz = QtWidgets.QSpinBox()
        self.time_tz.setRange(-128, 127)
        self.time_tz.setValue(0)
        self.time_tz.setToolTip("tz_q15（s8）。通常为时区偏移(UTC)的 15 分钟单位。")
        g.addWidget(self.time_tz, 1, 2)
        self.time_btn = QtWidgets.QPushButton("同步PC时间")
        self.time_btn.clicked.connect(self._on_time_sync)
        g.addWidget(self.time_btn, 1, 3)

        # Direction control: press move, release stop
        g.addWidget(QtWidgets.QLabel("电机方向 (0x22, 按下动/松开停)"), 2, 0, 1, 4)
        self.dir_speed = QtWidgets.QSpinBox()
        self.dir_speed.setRange(0, 3)
        self.dir_speed.setValue(2)
        g.addWidget(QtWidgets.QLabel("speed"), 3, 0)
        g.addWidget(self.dir_speed, 3, 1)
        self.btn_up = QtWidgets.QPushButton("↑")
        self.btn_down = QtWidgets.QPushButton("↓")
        self.btn_left = QtWidgets.QPushButton("←")
        self.btn_right = QtWidgets.QPushButton("→")
        g.addWidget(self.btn_up, 4, 1)
        g.addWidget(self.btn_left, 5, 0)
        g.addWidget(self.btn_down, 5, 1)
        g.addWidget(self.btn_right, 5, 2)
        self._bind_press_release(self.btn_up, 0)
        self._bind_press_release(self.btn_down, 1)
        self._bind_press_release(self.btn_left, 2)
        self._bind_press_release(self.btn_right, 3)

        # 设置高度
        g.addWidget(QtWidgets.QLabel("设置高度 (0x50)"), 6, 0, 1, 4)
        g.addWidget(QtWidgets.QLabel("高度(mm):"), 7, 0)
        self.new_h_mm = QtWidgets.QSpinBox()
        self.new_h_mm.setRange(500, 10000)
        self.new_h_mm.setValue(2500)
        g.addWidget(self.new_h_mm, 7, 1)
        b_config_height = QtWidgets.QPushButton("设置高度")
        b_config_height.clicked.connect(self._on_new_config_step1)
        g.addWidget(b_config_height, 7, 2)

        # ===== 狩猎游戏设置 (0x60-0x66) =====
        g.addWidget(QtWidgets.QLabel("狩猎设置 (0x60~0x66)"), 8, 0, 1, 4)
        row = 9

        def _nrow(inc=1):
            nonlocal row
            r = row
            row += inc
            return r

        # 进入/退出设置
        b_hunt_enter = QtWidgets.QPushButton("进入设置(0x60)")
        b_hunt_enter.clicked.connect(lambda: self._send(*vc.cmd_hunt_settings_enter()))
        b_hunt_exit_apply = QtWidgets.QPushButton("退出并应用(0x61)")
        b_hunt_exit_apply.clicked.connect(lambda: self._send(*vc.cmd_hunt_settings_exit(True)))
        b_hunt_exit_discard = QtWidgets.QPushButton("退出丢弃(0x61)")
        b_hunt_exit_discard.clicked.connect(lambda: self._send(*vc.cmd_hunt_settings_exit(False)))
        g.addWidget(QtWidgets.QLabel("设置模式"), _nrow(), 0)
        g.addWidget(b_hunt_enter, row-1, 1)
        g.addWidget(b_hunt_exit_apply, row-1, 2)
        g.addWidget(b_hunt_exit_discard, row-1, 3)

        # 猎物点操作
        b_prey_random_start = QtWidgets.QPushButton("开始随机(0x62)")
        b_prey_random_start.clicked.connect(lambda: self._send(*vc.cmd_hunt_prey_random(True)))
        b_prey_random_stop = QtWidgets.QPushButton("停止随机(0x62)")
        b_prey_random_stop.clicked.connect(lambda: self._send(*vc.cmd_hunt_prey_random(False)))
        g.addWidget(QtWidgets.QLabel("猎物点"), _nrow(), 0)
        g.addWidget(b_prey_random_start, row-1, 1)
        g.addWidget(b_prey_random_stop, row-1, 2)

        # 统一设置狩猎参数
        g.addWidget(QtWidgets.QLabel("狩猎参数设置(0x63)"), _nrow(), 0, 1, 4)
        self.hunt_dur_spin = QtWidgets.QSpinBox()
        self.hunt_dur_spin.setRange(10, 600)
        self.hunt_dur_spin.setValue(60)
        self.hunt_cnt_spin = QtWidgets.QSpinBox()
        self.hunt_cnt_spin.setRange(1, 60)
        self.hunt_cnt_spin.setValue(3)
        self.hunt_sleep_spin = QtWidgets.QSpinBox()
        self.hunt_sleep_spin.setRange(1, 20)
        self.hunt_sleep_spin.setValue(3)
        g.addWidget(QtWidgets.QLabel("时长(秒)"), _nrow(), 0)
        g.addWidget(self.hunt_dur_spin, row-1, 1)
        g.addWidget(QtWidgets.QLabel("次数"), row-1, 2)
        g.addWidget(self.hunt_cnt_spin, row-1, 3)
        g.addWidget(QtWidgets.QLabel("休眠(分)"), _nrow(), 0)
        g.addWidget(self.hunt_sleep_spin, row-1, 1)
        b_hunt_set_all = QtWidgets.QPushButton("统一设置(0x63)")
        b_hunt_set_all.setStyleSheet(
            "QPushButton { background: #585b70; font-weight: bold; }"
            "QPushButton:hover { background: #6c7086; }"
        )
        b_hunt_set_all.clicked.connect(
            lambda: self._send(*vc.cmd_hunt_settings_set(
                self.hunt_dur_spin.value(), self.hunt_cnt_spin.value(), self.hunt_sleep_spin.value()
            ))
        )
        g.addWidget(b_hunt_set_all, row-1, 2, 1, 2)

        # 获取当前设置
        b_hunt_get = QtWidgets.QPushButton("获取设置(0x64)")
        b_hunt_get.setStyleSheet(
            "QPushButton { background: #585b70; font-weight: bold; }"
            "QPushButton:hover { background: #6c7086; }"
        )
        b_hunt_get.clicked.connect(lambda: self._send(*vc.cmd_hunt_settings_get()))
        b_hunt_get.setToolTip("获取当前狩猎时长、次数、休眠时长")
        g.addWidget(b_hunt_get, _nrow(), 0, 1, 4)

        # 逗宠记录
        g.addWidget(QtWidgets.QLabel("逗宠记录 (0x33)"), _nrow(), 0, 1, 4)
        self.play_record_info = QtWidgets.QLabel("尚未收到记录")
        self.play_record_info.setStyleSheet("color: #a6e3a1; font-size: 11px;")
        g.addWidget(self.play_record_info, row, 0, 1, 4)
        _nrow()

        # 已收到的记录列表
        self.play_record_list = QtWidgets.QLabel("")
        self.play_record_list.setStyleSheet("color: #89b4fa; font-size: 10px;")
        self.play_record_list.setWordWrap(True)
        g.addWidget(self.play_record_list, row, 0, 1, 4)
        _nrow()

        b_play_record_ack = QtWidgets.QPushButton("确认收到 (ACK)")
        b_play_record_ack.setStyleSheet(
            "QPushButton { background: #45475a; font-weight: bold; color: #a6e3a1; }"
            "QPushButton:hover { background: #585b70; }"
        )
        b_play_record_ack.clicked.connect(self._on_play_record_ack)
        b_play_record_ack.setToolTip(
            "告知设备已收到当前逗宠记录，设备将在 1 秒后发送下一条（如有）"
        )
        g.addWidget(b_play_record_ack, row, 0, 1, 4)
        _nrow()

        # 删除指定记录 (0x35)
        g.addWidget(QtWidgets.QLabel("删除记录 (0x35):"), row, 0, 1, 1)
        self.play_record_del_id = QtWidgets.QSpinBox()
        self.play_record_del_id.setRange(0, 255)
        self.play_record_del_id.setValue(1)
        self.play_record_del_id.setStyleSheet("color: #cdd6f4; background: #313244;")
        g.addWidget(self.play_record_del_id, row, 1, 1, 1)
        b_play_record_del = QtWidgets.QPushButton("删除")
        b_play_record_del.setStyleSheet(
            "QPushButton { background: #f38ba8; font-weight: bold; }"
            "QPushButton:hover { background: #eba0ac; }"
        )
        b_play_record_del.clicked.connect(self._on_play_record_delete)
        b_play_record_del.setToolTip("删除指定ID的逗宠记录（需先收到记录）")
        g.addWidget(b_play_record_del, row, 2, 1, 2)
        _nrow()

        # 复位和重启
        g.addWidget(QtWidgets.QLabel("系统控制"), _nrow(), 0, 1, 4)
        b_reset = QtWidgets.QPushButton("复位配置(0x56)")
        b_reset.clicked.connect(lambda: self._send(*vc.cmd_radar_reset_flash()))
        g.addWidget(b_reset, row, 0, 1, 2)
        _nrow()

        b_reboot = QtWidgets.QPushButton("重启MCU(0x5A)")
        b_reboot.clicked.connect(lambda: self._send(*vc.cmd_device_reboot()))
        b_reboot.setToolTip("发送后设备会断开并重新启动（响应可能来不及到达）")
        g.addWidget(b_reboot, row-1, 2, 1, 2)

        # ===== OTA 固件升级 =====
        g.addWidget(QtWidgets.QLabel("OTA 固件升级"), _nrow(), 0, 1, 4)
        b_ota_version = QtWidgets.QPushButton("查询版本(0xFF00)")
        b_ota_version.setToolTip("发送 CMD_OTA_VERSION 请求，设备需注册 ota_versionCb_t 回调才能响应")
        b_ota_version.clicked.connect(self._on_ota_version_req)
        g.addWidget(b_ota_version, row, 0, 1, 2)
        _nrow()

        b_ota_select = QtWidgets.QPushButton("选择固件(.bin)")
        b_ota_select.clicked.connect(self._on_ota_select_file)
        g.addWidget(b_ota_select, row-1, 2, 1, 2)

        self._ota_fw_path_label = QtWidgets.QLabel("未选择固件文件")
        self._ota_fw_path_label.setStyleSheet("color: #f9e2af; font-size: 11px;")
        g.addWidget(self._ota_fw_path_label, _nrow(), 0, 1, 4)

        self._ota_start_btn = QtWidgets.QPushButton("开始 OTA 升级")
        self._ota_start_btn.setStyleSheet(
            "QPushButton { background: #a6e3a1; color: #11111b; font-weight: bold; }"
            "QPushButton:hover { background: #89dceb; }"
            "QPushButton:disabled { background: #45475a; color: #6c7086; }"
        )
        self._ota_start_btn.setEnabled(False)
        self._ota_start_btn.setToolTip("请先选择固件文件 (.bin)")
        self._ota_start_btn.clicked.connect(self._on_ota_start)
        g.addWidget(self._ota_start_btn, _nrow(), 0, 1, 4)

        self._ota_progress_bar = QtWidgets.QProgressBar()
        self._ota_progress_bar.setRange(0, 100)
        self._ota_progress_bar.setValue(0)
        self._ota_progress_bar.setTextVisible(True)
        self._ota_progress_bar.setStyleSheet(
            "QProgressBar { background: #313244; border: 1px solid #45475a; border-radius: 4px; text-align: center; color: #cdd6f4; }"
            "QProgressBar::chunk { background: #89b4fa; border-radius: 4px; }"
        )
        g.addWidget(self._ota_progress_bar, _nrow(), 0, 1, 4)

        self._ota_status_label = QtWidgets.QLabel("")
        self._ota_status_label.setStyleSheet("color: #a6e3a1; font-size: 11px;")
        self._ota_status_label.setWordWrap(True)
        g.addWidget(self._ota_status_label, row, 0, 1, 4)
        _nrow()

        if self.vis._transport != "ble":
            self.time_use_local_tz.setEnabled(False)
            self.time_tz.setEnabled(False)
            self.time_btn.setEnabled(False)
            self.time_btn.setToolTip("仅 BLE 模式可下发")

    def _on_time_sync(self) -> None:
        tz_q15 = self.time_tz.value()
        if self.time_use_local_tz.isChecked():
            try:
                now = datetime.datetime.now(datetime.timezone.utc).astimezone()
                off = now.utcoffset()
                off_sec = int(off.total_seconds()) if off else 0
                tz_q15 = int(round(off_sec / 900.0))
                tz_q15 = max(-128, min(127, tz_q15))
                self.time_tz.setValue(tz_q15)
            except Exception:
                pass
        epoch_sec = int(time.time())
        ok = self.vis.send_cmd(*vc.cmd_time_set(epoch_sec, tz_q15))
        if not ok:
            self.log("下发失败（非 BLE 或未连接）")

    def _on_new_config_step1(self) -> None:
        height_mm = self.new_h_mm.value()
        ok = self.vis.send_cmd(*vc.cmd_radar_config_set_height(height_mm))
        if not ok:
            self.log("下发失败（非 BLE 或未连接）")
        else:
            self.log(f"已发送：设置高度 {height_mm}mm (0x50)")

    def _on_play_record_ack(self) -> None:
        ok = self.vis.send_cmd(*vc.cmd_play_record_ack())
        if not ok:
            self.log("ACK 下发失败（非 BLE 或未连接）")
        else:
            self.log("已发送逗宠记录 ACK (0x33)")

    def _on_play_record_delete(self) -> None:
        record_id = self.play_record_del_id.value()
        ok = self.vis.send_cmd(*vc.cmd_play_record_delete(record_id))
        if not ok:
            self.log(f"删除记录 ID={record_id} 下发失败（非 BLE 或未连接）")
        else:
            self.log(f"已发送删除逗宠记录 ID={record_id} (0x35)")

    # ===== OTA 操作 =====
    _ota_selected_fw: str = ""

    def _on_ota_version_req(self) -> None:
        """发送 CMD_OTA_VERSION (0xFF00) 请求从设备获取固件版本。"""
        if self.vis._transport != "ble":
            self.log("OTA 命令仅支持 BLE 模式")
            return
        self.vis.send_ota_version_request()
        self.log("已发送 CMD_OTA_VERSION (0xFF00)（仅发送命令，不阻塞 OTA 升级按钮）")

    def _on_ota_select_file(self) -> None:
        default_path = r"C:\Users\JampMar\Desktop\code\qiuqiu\tc_ble_simple_sdk_B80_V3.4.2.2_P10\b80_ble_sdk\cmake_builds\tc_ble_simple_b80_sdk\TC32-GCC_Toolchain\b80_ble_cat_laser_toy.bin"
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择固件文件 (.bin)", default_path, "BIN 文件 (*.bin)"
        )
        if not path:
            return
        self._ota_selected_fw = path
        self._ota_fw_path_label.setText(os.path.basename(path))
        self._ota_fw_path_label.setToolTip(path)
        self._ota_start_btn.setEnabled(True)
        self._ota_start_btn.setToolTip(f"OTA: {os.path.basename(path)}")
        self.log(f"已选择固件: {path}")

    def _on_ota_start(self) -> None:
        if not self._ota_selected_fw:
            return
        if not os.path.isfile(self._ota_selected_fw):
            self.log(f"固件文件不存在: {self._ota_selected_fw}")
            return
        if self.vis._transport != "ble":
            self.log("OTA 仅支持 BLE 模式")
            return
        ok = self.vis.start_ota(self._ota_selected_fw)
        if ok:
            self._ota_start_btn.setEnabled(False)
            self.log(f"OTA 已启动: {self._ota_selected_fw}")
        else:
            self.log("OTA 启动失败（可能正在运行中）")

    def _bind_press_release(self, btn: QtWidgets.QPushButton, direction: int) -> None:
        btn.pressed.connect(
            lambda d=direction: self._send(*vc.cmd_motor_dir_start(d, self.dir_speed.value()))
        )
        btn.released.connect(
            lambda d=direction: self._send(*vc.cmd_motor_dir_stop(d, self.dir_speed.value()))
        )

    def _stop_auto_power_if_running(self) -> None:
        if self._auto_power_timer.isActive():
            self._auto_power_timer.stop()
            self._auto_power_btn.setChecked(False)
            self.log("手动操作中断自动开关循环")

    def _on_auto_power_toggle(self) -> None:
        btn = self._auto_power_btn
        if btn.isChecked():
            self._auto_power_on_phase = True
            self._auto_power_timer.start()
            self._send(*vc.cmd_power_ctrl(1))
            self.log(f"开始循环：{self._auto_power_interval}s开→{self._auto_power_interval}s关")
        else:
            self._auto_power_timer.stop()
            self.log("自动开关已停止")

    def _on_auto_power_tick(self) -> None:
        if self._auto_power_on_phase:
            self._send(*vc.cmd_power_ctrl(0))
            self._auto_power_on_phase = False
        else:
            self._send(*vc.cmd_power_ctrl(1))
            self._auto_power_on_phase = True

    def _on_svc_update_tick(self) -> None:
        # Update play record info from vis state
        with self.vis._lock:
            rec_info = self.vis._play_record_info
            records = list(self.vis._play_records)  # copy under lock
        cur_text = self.play_record_info.text()
        if rec_info and rec_info != cur_text:
            self.play_record_info.setText(rec_info)

        # Update record list display
        if records:
            lines = []
            for rec in records[-8:]:  # show up to 8 most recent
                rid, start_str, end_str, motion, speed, result_str = rec
                lines.append(f"ID={rid}: {start_str}~{end_str} {motion}s {speed}cm/s {result_str}")
            self.play_record_list.setText("\n".join(lines))

        # Check BLE disconnect for auto power stop
        with self.vis._lock:
            need_stop = self.vis._ble_disconnect_need_stop_auto_power
            if need_stop:
                self.vis._ble_disconnect_need_stop_auto_power = False
        if need_stop:
            self._stop_auto_power_if_running()
            self.log("断开连接，自动开关机已停止")

        # Update OTA status
        ota_running, ota_progress, ota_status = self.vis.ota_state()
        if ota_running:
            self._ota_progress_bar.setValue(int(ota_progress * 100))
            self._ota_status_label.setText(ota_status)
            self._ota_start_btn.setEnabled(False)
        else:
            # Only update when there's a status to show
            if ota_status:
                self._ota_progress_bar.setValue(int(ota_progress * 100))
                self._ota_status_label.setText(ota_status)
            # Re-enable OTA button if OTA completed (progress >= 1.0)
            if ota_progress >= 1.0 and ota_status and "失败" not in ota_status:
                self._ota_start_btn.setEnabled(bool(self._ota_selected_fw))


class RadarNightWindow(QtWidgets.QMainWindow):
    """夜间模式：左侧坐标图，右侧雷达数据 + 跟踪步间隔下发 (0x58)。"""

    def __init__(self, vis: RadarVisualizer, title: str):
        super().__init__()
        self.vis = vis
        self._last_sector_epoch = -1
        self._last_ota_epoch = -1

        self.setWindowTitle(title)
        self.resize(1180, 820)
        self.setStyleSheet(NIGHT_STYLESHEET)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)

        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        root.addWidget(splitter, 1)

        left_wrap = QtWidgets.QWidget()
        left_l = QtWidgets.QVBoxLayout(left_wrap)
        left_l.setContentsMargins(0, 0, 0, 0)
        self.figure = Figure(figsize=(6.5, 8), dpi=100, facecolor="#11111b")
        self.canvas = FigureCanvas(self.figure)
        left_l.addWidget(self.canvas, 1)
        splitter.addWidget(left_wrap)

        right = QtWidgets.QWidget()
        right_l = QtWidgets.QVBoxLayout(right)
        right_l.setContentsMargins(0, 0, 0, 0)
        right_l.setSpacing(10)

        lbl_data = QtWidgets.QLabel("雷达数据")
        lbl_data.setStyleSheet("font-size: 14px; color: #89b4fa; font-weight: bold;")
        right_l.addWidget(lbl_data)
        self.radar_text = QtWidgets.QPlainTextEdit()
        self.radar_text.setReadOnly(True)
        self.radar_text.setMinimumWidth(340)
        self.radar_text.document().setDefaultFont(
            QtGui.QFont("Consolas", 11)
            if sys.platform == "win32"
            else QtGui.QFont("monospace", 11)
        )
        right_l.addWidget(self.radar_text, 1)

        lbl_log = QtWidgets.QLabel("固件日志 (Log TX / 0x40)")
        lbl_log.setStyleSheet("font-size: 14px; color: #89b4fa; font-weight: bold;")
        right_l.addWidget(lbl_log)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(170)
        self.log_text.document().setDefaultFont(
            QtGui.QFont("Consolas", 10)
            if sys.platform == "win32"
            else QtGui.QFont("monospace", 10)
        )
        right_l.addWidget(self.log_text, 1)

        speed_box = QtWidgets.QGroupBox("跟踪速度 (BLE → 设备)")
        speed_l = QtWidgets.QVBoxLayout(speed_box)
        hint = QtWidgets.QLabel(
            "步进间隔 interval_us：数值越小电机越快（固件与 StepMotor_GimbalSetSpeedUs 一致）。"
            f" 允许 {TRACK_INTERVAL_US_MIN}～{TRACK_INTERVAL_US_MAX} µs。"
        )
        hint.setWordWrap(True)
        speed_l.addWidget(hint)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("interval_us:"))
        self.speed_spin = QtWidgets.QSpinBox()
        self.speed_spin.setRange(TRACK_INTERVAL_US_MIN, TRACK_INTERVAL_US_MAX)
        self.speed_spin.setValue(TRACK_INTERVAL_DEFAULT_US)
        self.speed_spin.setSingleStep(25)
        row.addWidget(self.speed_spin, 1)
        speed_l.addLayout(row)
        self.speed_btn = QtWidgets.QPushButton("下发 CMD 0x58 (RADAR_TRACK_SPEED)")
        self.speed_btn.clicked.connect(self._on_send_speed)
        speed_l.addWidget(self.speed_btn)
        if vis._transport != "ble":
            self.speed_spin.setEnabled(False)
            self.speed_btn.setEnabled(False)
            self.speed_btn.setToolTip("仅 BLE 模式可下发")
        right_l.addWidget(speed_box)

        # --- 云台偏移角度控制 ---
        pan_box = QtWidgets.QGroupBox("云台偏移角度")
        pan_l = QtWidgets.QVBoxLayout(pan_box)
        row_p = QtWidgets.QHBoxLayout()
        row_p.addWidget(QtWidgets.QLabel("pan_deg10:"))
        self.pan_spin = QtWidgets.QSpinBox()
        self.pan_spin.setRange(-3000, 3000)
        self.pan_spin.setValue(-50)
        self.pan_spin.setSingleStep(10)
        row_p.addWidget(self.pan_spin, 1)
        pan_l.addLayout(row_p)
        row_t = QtWidgets.QHBoxLayout()
        row_t.addWidget(QtWidgets.QLabel("tilt_deg10:"))
        self.tilt_spin = QtWidgets.QSpinBox()
        self.tilt_spin.setRange(-3000, 3000)
        self.tilt_spin.setValue(0)
        self.tilt_spin.setSingleStep(10)
        row_t.addWidget(self.tilt_spin, 1)
        pan_l.addLayout(row_t)
        self.pan_btn = QtWidgets.QPushButton("下发 CMD 0x51 (PAN+TILT)")
        self.pan_btn.clicked.connect(self._on_send_pan_tilt_offset)
        pan_l.addWidget(self.pan_btn)
        if vis._transport != "ble":
            self.pan_spin.setEnabled(False)
            self.tilt_spin.setEnabled(False)
            self.pan_btn.setEnabled(False)
            self.pan_btn.setToolTip("仅 BLE 模式可下发")
        right_l.addWidget(pan_box)

        # --- 固件版本 & OTA 状态 ---
        fw_box = QtWidgets.QGroupBox("固件 / OTA")
        fw_l = QtWidgets.QVBoxLayout(fw_box)
        self.fw_label = QtWidgets.QLabel("FW: —")
        self.fw_label.setStyleSheet("color: #a6e3a1; font-weight: bold;")
        fw_l.addWidget(self.fw_label)
        self.ota_label = QtWidgets.QLabel("OTA: —")
        self.ota_label.setStyleSheet("color: #f9e2af;")
        fw_l.addWidget(self.ota_label)
        right_l.addWidget(fw_box)

        conn_box = QtWidgets.QGroupBox("BLE 连接")
        conn_l = QtWidgets.QVBoxLayout(conn_box)
        row_addr = QtWidgets.QHBoxLayout()
        row_addr.addWidget(QtWidgets.QLabel("address:"))
        self.addr_edit = QtWidgets.QLineEdit()
        cur_addr, cur_en = (
            self.vis.ble_target() if self.vis._transport == "ble" else ("", False)
        )
        self.addr_edit.setText(cur_addr)
        row_addr.addWidget(self.addr_edit, 1)
        conn_l.addLayout(row_addr)
        self.conn_btn = QtWidgets.QPushButton("连接")
        self.conn_btn.setCheckable(True)
        self.conn_btn.setChecked(cur_en)
        self._update_conn_btn_text()
        self.conn_btn.clicked.connect(self._on_toggle_ble_conn)
        conn_l.addWidget(self.conn_btn)

        # BLE 断开次数显示
        self.disconnect_count_label = QtWidgets.QLabel("断开连接次数: 0")
        self.disconnect_count_label.setStyleSheet("color: #f38ba8; font-size: 12px; font-weight: bold;")
        conn_l.addWidget(self.disconnect_count_label)

        if vis._transport != "ble":
            self.addr_edit.setEnabled(False)
            self.conn_btn.setEnabled(False)
            self.conn_btn.setToolTip("仅 BLE 模式可用")
        right_l.addWidget(conn_box)

        self._ctrl_svc_window = None
        self._open_svc_btn = QtWidgets.QPushButton("打开控制服务面板")
        self._open_svc_btn.clicked.connect(self._open_ctrl_service_window)
        right_l.addWidget(self._open_svc_btn)

        self.ctrl_text = QtWidgets.QPlainTextEdit()
        self.ctrl_text.setReadOnly(True)
        self.ctrl_text.setMinimumHeight(130)
        self.ctrl_text.setPlaceholderText("Ctrl RSP / EVENT 日志")
        right_l.addWidget(self.ctrl_text, 1)

        splitter.addWidget(right)
        splitter.setSizes([720, 420])

        self.ax = self.figure.add_subplot(111, facecolor="#181825")
        self._setup_plot()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(80)
        self.timer.timeout.connect(self._update_view)
        self.timer.start()

        self.log_timer = QtCore.QTimer(self)
        self.log_timer.setInterval(60)
        self.log_timer.timeout.connect(self._flush_raw_log)
        self.log_timer.start()

        self.ctrl_timer = QtCore.QTimer(self)
        self.ctrl_timer.setInterval(80)
        self.ctrl_timer.timeout.connect(self._flush_ctrl_log)
        self.ctrl_timer.start()

    def _on_send_pan_tilt_offset(self) -> None:
        ok = self.vis.send_radar_pan_tilt_offset_deg10(
            self.pan_spin.value(), self.tilt_spin.value()
        )
        if not ok:
            self.radar_text.appendPlainText("[本地] 下发失败（非 BLE 或未连接）\n")
        else:
            self.radar_text.appendPlainText(
                f"[本地] 下发 PAN={self.pan_spin.value()}  TILT={self.tilt_spin.value()} deg10\n"
            )

    def _on_send_speed(self) -> None:
        ok = self.vis.send_radar_track_interval_us(self.speed_spin.value())
        if not ok:
            self.radar_text.appendPlainText("[本地] 下发失败（非 BLE 或未连接）\n")

    def _update_conn_btn_text(self) -> None:
        self.conn_btn.setText("断开" if self.conn_btn.isChecked() else "连接")

    def _on_toggle_ble_conn(self) -> None:
        enabled = self.conn_btn.isChecked()
        addr = self.addr_edit.text().strip()
        if enabled and not addr:
            self.conn_btn.setChecked(False)
            self._update_conn_btn_text()
            self.radar_text.appendPlainText("[本地] 请输入 BLE address")
            return
        self.vis.set_ble_target(addr, enabled)
        self._update_conn_btn_text()
        self.radar_text.appendPlainText(
            f"[本地] BLE {'连接请求' if enabled else '已断开'}: {addr}"
        )

    def _open_ctrl_service_window(self) -> None:
        if self._ctrl_svc_window is None:
            self._ctrl_svc_window = CtrlServiceWindow(self.vis, self)
        self._ctrl_svc_window.show()
        self._ctrl_svc_window.raise_()

    def _setup_plot(self):
        C_TEXT = "#cdd6f4"
        C_GRID = "#45475a"
        self.ax.tick_params(colors=C_TEXT)
        for s in self.ax.spines.values():
            s.set_color(C_GRID)
        self.ax.xaxis.label.set_color(C_TEXT)
        self.ax.yaxis.label.set_color(C_TEXT)
        self.ax.title.set_color("#89b4fa")

        # 环形扇区：由 2 条弧线 + 2 条径向线组成
        self.boundary_artists = []  # [inner_arc, outer_arc, left_radial, right_radial]
        for _ in range(4):
            (line,) = self.ax.plot([], [], color="#89b4fa", linewidth=2, alpha=0.95)
            self.boundary_artists.append(line)
        self._sector_artist_epoch = -1

        # 4 象限分界线：x=0（垂直）和 y=ro*0.5（水平）
        (self.div_v_line,) = self.ax.plot(
            [], [], color="#f9e2af", linewidth=1, linestyle="--", alpha=0.5, label="Divider"
        )
        (self.div_h_line,) = self.ax.plot(
            [], [], color="#f9e2af", linewidth=1, linestyle="--", alpha=0.5
        )

        (self.prev_point,) = self.ax.plot(
            [], [], marker="x", color="#f9e2af", markersize=9, mew=2, label="Prev"
        )
        (self.raw_point,) = self.ax.plot(
            [], [], "o", color="#89dceb", markersize=7, label="Raw"
        )
        (self.seq_points,) = self.ax.plot(
            [], [], ".", color="#f38ba8", alpha=0.85, markersize=8, label="Track / Seq"
        )

        self.raw_text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            va="top",
            fontsize=9,
            color="#bac2de",
        )

        self.ax.set_xlabel("X (mm)", color=C_TEXT)
        self.ax.set_ylabel("Y (mm)", color=C_TEXT)
        self.ax.set_xlim(X_MIN - 100, X_MAX + 100)
        self.ax.set_ylim(Y_MIN - 100, Y_MAX + 100)
        self.ax.grid(True, linestyle="--", alpha=0.35, color=C_GRID)

        self.motion_arrow = FancyArrowPatch(
            (0.0, 0.0),
            (0.0, 0.0),
            arrowstyle="-|>",
            mutation_scale=28,
            linewidth=3.5,
            edgecolor="#f5c2e7",
            facecolor="#f5c2e7",
            zorder=9,
            label="Motion",
        )
        self.ax.add_patch(self.motion_arrow)
        self.motion_arrow.set_visible(False)

        leg = self.ax.legend(loc="lower right", facecolor="#1e1e2e", edgecolor=C_GRID)
        for t in leg.get_texts():
            t.set_color(C_TEXT)
        self.figure.tight_layout()

    def _flush_raw_log(self):
        with self.vis._lock:
            if not self.vis.log_lines:
                return
            lines = list(self.vis.log_lines)
            self.vis.log_lines.clear()
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        timestamped_lines = [f"[{now_str}] {l}" for l in lines]
        self.log_text.appendPlainText("\n".join(timestamped_lines))
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    def _flush_ctrl_log(self):
        with self.vis._lock:
            if not self.vis.ctrl_lines:
                return
            lines = list(self.vis.ctrl_lines)
            self.vis.ctrl_lines.clear()
        self.ctrl_text.appendPlainText("\n".join(lines))
        self.ctrl_text.verticalScrollBar().setValue(
            self.ctrl_text.verticalScrollBar().maximum()
        )

    @staticmethod
    def _build_sector_polygon(cx, cy, ri, ro, a_start_deg10, a_end_deg10, n_pts=60):
        """将环形扇区转为闭合多边形顶点列表 [(x,y),...] 用于 matplotlib 绘图"""
        a_start = math.radians(a_start_deg10 / 10.0)
        a_end = math.radians(a_end_deg10 / 10.0)
        # 沿内弧从 start 到 end
        angles = [a_start + (a_end - a_start) * i / n_pts for i in range(n_pts + 1)]
        pts = []
        # 内弧
        for a in angles:
            pts.append((cx + ri * math.sin(a), cy + ri * math.cos(a)))
        # 外弧（逆行）
        for a in reversed(angles):
            pts.append((cx + ro * math.sin(a), cy + ro * math.cos(a)))
        pts.append(pts[0])  # 闭合
        return pts

    def _update_view(self):
        with self.vis._lock:
            latest_prev = self.vis.latest_prev
            latest_raw = self.vis.latest_raw
            latest_a = self.vis.latest_pred_a
            latest_b = self.vis.latest_pred_b
            seq = list(self.vis.seq_history)
            sepoch = self.vis.sector_epoch
            sr = self.vis.sector_region  # (cx, cy, ri, ro, a_start, a_end)
            mdir_ok = self.vis.motion_dir_valid
            mdeg10 = self.vis.motion_dir_deg10
            ota_ep = self.vis.ota_status_epoch
            disconnect_count = self.vis._ble_disconnect_count
            last_disconnect_time = self.vis._ble_last_disconnect_time

        if sepoch != self._last_sector_epoch:
            self._last_sector_epoch = sepoch
            cx, cy, ri, ro, ast, aen = sr
            poly = self._build_sector_polygon(cx, cy, ri, ro, ast, aen)
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            # 用 4 条折线分别绘制内弧/外弧/左径向/右径向 → 只需一条闭合折线即可
            for i, line in enumerate(self.boundary_artists):
                if i == 0:
                    line.set_data(xs, ys)
                else:
                    line.set_data([], [])
            # 更新 4 象限分界线
            angle_rad = math.radians(ast / 10.0)
            split_y = ro * math.cos(angle_rad)  # = ro * cos(60°)
            ext = ro + 500  # 线超出扇区范围一些
            self.div_v_line.set_data([0, 0], [cy - ext, cy + ext])
            self.div_h_line.set_data([cx - ext, cx + ext], [split_y, split_y])

        if latest_prev is not None:
            self.prev_point.set_data([latest_prev[0]], [latest_prev[1]])
        else:
            self.prev_point.set_data([], [])

        if latest_raw is not None:
            x, y, v = latest_raw
            self.raw_point.set_data([x], [y])
            if mdir_ok and v is None:
                th = mdeg10 / 10.0
                self.raw_text.set_text(f"RAW x={x} y={y} mm  θ={th:.1f}°")
            elif v is None:
                self.raw_text.set_text(f"RAW x={x} y={y} mm")
            else:
                self.raw_text.set_text(f"RAW x={x} y={y} mm  v={v} cm/s")
            if mdir_ok:
                rad = math.radians(mdeg10 / 10.0)
                ux = math.sin(rad)
                uy = math.cos(rad)
                x2 = float(x) + MOTION_ARROW_MM * ux
                y2 = float(y) + MOTION_ARROW_MM * uy
                self.motion_arrow.set_positions((float(x), float(y)), (x2, y2))
                self.motion_arrow.set_visible(True)
            else:
                self.motion_arrow.set_visible(False)
        else:
            self.raw_point.set_data([], [])
            self.raw_text.set_text("RAW: —")
            self.motion_arrow.set_visible(False)

        # 更新固件版本 & OTA 状态
        with self.vis._lock:
            fw = self.vis.fw_version
            ota_st = self.vis.ota_status
            ota_ep = self.vis.ota_status_epoch
        if fw:
            self.fw_label.setText(f"FW: v{fw}")
        if ota_ep != self._last_ota_epoch:
            self._last_ota_epoch = ota_ep
            ota_text = {0: "空闲", 1: "更新中…", 2: "更新成功 ✅", 3: "更新失败 ❌"}
            self.ota_label.setText(f"OTA: {ota_text.get(ota_st, '?')}")
            if ota_st in (2, 3):
                self.radar_text.appendPlainText(f"OTA 状态变更: {ota_text[ota_st]}")

        if seq:
            xs = [p[0] for p in seq]
            ys = [p[1] for p in seq]
            self.seq_points.set_data(xs, ys)
        else:
            self.seq_points.set_data([], [])

        lines = [
            "── 目标点 ──",
            f"Prev:     {latest_prev if latest_prev else '—'}",
            f"Raw:      {latest_raw[:2] if latest_raw else '—'}",
            f"Motion:   valid={mdir_ok}  dir_deg×10={mdeg10}",
            "",
            "── 预测 / 跟踪 ──",
            f"Seq pts:  {len(seq)}  {seq[-3:] if seq else ''}",
            "",
            "── 场地 (环形扇区) ──",
            f"Sector epoch: {sepoch}",
            f"Sector: cx={sr[0]} cy={sr[1]} ri={sr[2]} ro={sr[3]} aStart={sr[4]} aEnd={sr[5]}",
            "",
            f"── BLE ──",
            f"断开连接次数: {disconnect_count}",
            f"最近断开时间: {last_disconnect_time if last_disconnect_time else '--'}",
            "",
            f"UI 待下发 interval: {self.speed_spin.value()} µs  (CMD 0x58)",
        ]
        self.radar_text.setPlainText("\n".join(lines))

        # 更新 BLE 断开次数标签
        disc_time_str = last_disconnect_time if last_disconnect_time else "--"
        self.disconnect_count_label.setText(f"断开连接次数: {disconnect_count}  最近: {disc_time_str}")

        self.canvas.draw_idle()


# python visializable.py --transport ble --address A4:C1:38:9F:96:BB
def main():
    parser = argparse.ArgumentParser(
        description="Radar prediction debug (serial or BLE)"
    )
    parser.add_argument(
        "--transport",
        choices=("ble", "serial"),
        default="ble",
        help="Data source (default: ble)",
    )
    parser.add_argument(
        "--port", default=PORT, help="Serial port when using --transport serial"
    )
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument(
        "--address",
        default="",
        help="BLE device address (Windows e.g. AA:BB:CC:DD:EE:FF). Required for BLE.",
    )
    parser.add_argument(
        "--ble-list-gatt",
        action="store_true",
        help="After connect, print full GATT table to stderr (debug).",
    )
    parser.add_argument(
        "--no-ble-request-boundary",
        action="store_true",
        help="Do not auto-write CMD 0x58 on connect (boundary only if you trigger manually).",
    )
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(NIGHT_STYLESHEET)
    vis = RadarVisualizer(
        args.transport,
        port=args.port,
        baudrate=args.baudrate,
        ble_address=args.address or None,
        ble_list_gatt=args.ble_list_gatt,
        ble_request_boundary=(not args.no_ble_request_boundary),
    )
    vis.start()

    if args.transport == "ble":
        win_title = f"Radar visualizer (BLE {args.address or 'manual'})"
    else:
        win_title = f"Radar visualizer (serial {args.port})"

    main_window = RadarNightWindow(vis, win_title)
    main_window.show()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        vis.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
