"""
Radar prediction debug visualizer.

- Serial: text lines after 8-char log prefix, e.g. PREV,...,RAW,... / PRED,STA,... / PREDSEQ,...
- BLE: binary EVENT on Ctrl TX (cmd 0x57): prediction debug + BOUNDARY_PT (0x04).
  BOUNDARY_PT is **not** streamed: device sends 4 NOTIFY only after CMD **0x58** on Ctrl RX.
  **CMD 0x59** (RADAR_TRACK_SPEED): APP writes Ctrl RX with payload u16 LE ``interval_us`` (µs)
  to set radar track gimbal step interval (same as ``StepMotor_GimbalSetSpeedUs``).
  The night-mode UI can send 0x59 from the right panel.
  Script defaults to requesting the quad on connect; use --no-ble-request-boundary to skip.
  The UI updates the black quad only after **all four** corners are received.
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
import math
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

CTRL_PROTO_VERSION = 0x01
CTRL_MSG_TYPE_CMD = 0x01
CTRL_MSG_TYPE_EVENT = 0x03
CTRL_CMD_RADAR_PRED_DEBUG = 0x57
CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY = 0x58
CTRL_CMD_RADAR_TRACK_SPEED = 0x59

CTRL_RADAR_DBG_SUB_PREV_RAW = 0x01
CTRL_RADAR_DBG_SUB_PRED_STA = 0x02
CTRL_RADAR_DBG_SUB_PREDSEQ = 0x03
CTRL_RADAR_DBG_SUB_BOUNDARY_PT = 0x04

# x: [-1100, 1100], y: [100, 4100]
X_MIN, X_MAX = -2500, 2500
Y_MIN, Y_MAX = 100, 7000

# From RAW point, arrow length in mm (matches firmware sin/cos(motion_rad) step convention)
MOTION_ARROW_MM = 480.0

BOUNDARY_QUAD = [
    (-1000, 800),
    (1000, 800),
    (1000, 4000),
    (-1000, 4000),
]

# 与固件 StepMotor_ClampIntervalUs 下限一致；上位机发送前也会夹紧
TRACK_INTERVAL_US_MIN = 750
TRACK_INTERVAL_US_MAX = 20000
TRACK_INTERVAL_DEFAULT_US = 800


def build_ctrl_cmd_frame(cmd_id: int, seq: int, payload: bytes) -> bytes:
    """version, msgType=CMD, cmdId, seq, payLen u16 LE, payload."""
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

        self.boundary_quad: List[Tuple[int, int]] = [tuple(p) for p in BOUNDARY_QUAD]
        self.boundary_epoch = 0
        self._boundary_corner_rcv: List[Optional[Tuple[int, int]]] = [
            None,
            None,
            None,
            None,
        ]

        self.raw_lines = deque(maxlen=2000)

        if transport == "serial":
            self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

    def send_radar_track_interval_us(self, interval_us: int) -> bool:
        """下发 CTRL_CMD_RADAR_TRACK_SPEED (0x59)，payload u16 LE interval_us。仅 BLE。"""
        if self._transport != "ble":
            return False
        v = max(TRACK_INTERVAL_US_MIN, min(TRACK_INTERVAL_US_MAX, int(interval_us)))
        self._ble_tx_seq = (self._ble_tx_seq + 1) & 0xFF
        pl = struct.pack("<H", v)
        frame = build_ctrl_cmd_frame(CTRL_CMD_RADAR_TRACK_SPEED, self._ble_tx_seq, pl)
        self._ble_tx_queue.put(frame)
        return True

    def start(self):
        if self._transport == "serial":
            t = threading.Thread(target=self._reader_loop_serial, daemon=True)
            t.start()
        else:
            self._ble_thread = threading.Thread(target=self._ble_worker, daemon=True)
            self._ble_thread.start()

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
                    self.raw_lines.append(line)
                if len(line) > 8:
                    self._parse_line(line[8:])
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

        address = self._ble_address
        if not address:
            return

        def on_notify(_handle, data: bytearray):
            b = bytes(data)
            with self._lock:
                self.raw_lines.append(b.hex())
            self._apply_ble_frame(b)

        def on_disconnected(*_args):
            print("[BLE] stack reported disconnect", file=sys.stderr)

        # Reconnect loop: peripheral or host may drop link (supervision ~4s on this firmware).
        while not self._stop.is_set():
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
                    print(f"[BLE] notify on {ctrl_tx.uuid}", file=sys.stderr)
                    await client.start_notify(ctrl_tx, on_notify)
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
                                await client.write_gatt_char(rx_char, req, response=True)
                            except Exception:
                                await client.write_gatt_char(rx_char, req, response=False)

                        await asyncio.sleep(0.25)
                        try:
                            if rx_ch is None:
                                print(
                                    "[BLE] Ctrl RX not found; cannot request boundary quad",
                                    file=sys.stderr,
                                )
                            else:
                                await _write_get_boundary(rx_ch)
                                print("[BLE] sent GET_BOUNDARY (0x58)", file=sys.stderr)
                                await asyncio.sleep(1.0)
                                with self._lock:
                                    need_retry = self.boundary_epoch == 0
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
                    keepalive = 0
                    while not self._stop.is_set():
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
                    self._ble_ctrl_tx_char = None
                    self._ble_ctrl_rx_char = None
            except Exception as ex:
                if self._stop.is_set():
                    break
                print("[BLE] session error (reconnecting):", ex, file=sys.stderr)
                traceback.print_exc()
                await asyncio.sleep(1.5)

    def _apply_ble_frame(self, data: bytes) -> None:
        if len(data) < 6:
            return
        if data[0] != CTRL_PROTO_VERSION:
            return
        if data[1] != CTRL_MSG_TYPE_EVENT or data[2] != CTRL_CMD_RADAR_PRED_DEBUG:
            # RSP (0x02) and other CMDs also use Ctrl TX on some stacks; ignore here.
            return
        payload_len = data[4] | (data[5] << 8)
        if len(data) < 6 + payload_len:
            return
        payload = data[6 : 6 + payload_len]
        if not payload:
            return
        sub = payload[0]
        try:
            if sub not in (
                CTRL_RADAR_DBG_SUB_PREV_RAW,
                CTRL_RADAR_DBG_SUB_PRED_STA,
                CTRL_RADAR_DBG_SUB_PREDSEQ,
                CTRL_RADAR_DBG_SUB_BOUNDARY_PT,
            ):
                print(f"[BLE] unknown SUB in PRED_DEBUG payload: 0x{sub:02x}", file=sys.stderr)
            if sub == CTRL_RADAR_DBG_SUB_PREV_RAW and len(payload) >= 12:
                prev_x, prev_y, raw_x, raw_y, m_valid, m_deg10 = struct.unpack_from(
                    "<hhhhBh", payload, 1
                )
                self._apply_prev_raw(
                    prev_x, prev_y, raw_x, raw_y, int(m_valid), int(m_deg10)
                )
            elif sub == CTRL_RADAR_DBG_SUB_PREV_RAW and len(payload) >= 9:
                prev_x, prev_y, raw_x, raw_y = struct.unpack_from("<hhhh", payload, 1)
                self._apply_prev_raw(prev_x, prev_y, raw_x, raw_y, 0, 0)
            elif sub == CTRL_RADAR_DBG_SUB_PRED_STA and len(payload) >= 9:
                ax, ay, bx, by = struct.unpack_from("<hhhh", payload, 1)
                self._apply_pred_sta(ax, ay, bx, by)
            elif sub == CTRL_RADAR_DBG_SUB_PREDSEQ and len(payload) >= 6:
                idx = payload[1]
                x, y = struct.unpack_from("<hh", payload, 2)
                self._apply_predseq(idx, x, y)
            elif sub == CTRL_RADAR_DBG_SUB_BOUNDARY_PT and len(payload) >= 6:
                bidx, bx, by = struct.unpack_from("<Bhh", payload, 1)
                print(f"[BLE] BOUNDARY_PT corner={bidx} x={bx} y={by}", file=sys.stderr)
                self._apply_boundary_pt(bidx, bx, by)
        except struct.error:
            return

    def _apply_boundary_pt(self, corner_idx: int, x_mm: int, y_mm: int) -> None:
        if corner_idx < 0 or corner_idx > 3:
            return
        with self._lock:
            self._boundary_corner_rcv[corner_idx] = (x_mm, y_mm)
            if all(self._boundary_corner_rcv[i] is not None for i in range(4)):
                self.boundary_quad = []
                for i in range(4):
                    pt = self._boundary_corner_rcv[i]
                    assert pt is not None
                    self.boundary_quad.append(pt)
                self.boundary_epoch += 1
                self._boundary_corner_rcv = [None, None, None, None]

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

    def _parse_line(self, line: str):
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            return

        try:
            if parts[0] == "PREV" and len(parts) >= 6 and parts[3] == "RAW":
                self._apply_prev_raw(
                    int(parts[1]), int(parts[2]), int(parts[4]), int(parts[5])
                )
            elif parts[0] == "PRED" and parts[1] == "STA" and len(parts) >= 6:
                self._apply_pred_sta(
                    int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])
                )
            elif parts[0] == "PREDSEQ" and len(parts) >= 4:
                self._apply_predseq(int(parts[1]), int(parts[2]), int(parts[3]))
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


class RawLogWindow(QtWidgets.QMainWindow):
    def __init__(self, title: str = "Raw data"):
        super().__init__()
        self.setWindowTitle(title)
        self.resize(520, 600)
        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.setCentralWidget(self.text)
        self.setStyleSheet(NIGHT_STYLESHEET)

    def append_lines(self, lines):
        if not lines:
            return
        self.text.appendPlainText("\n".join(lines))
        self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())


class RadarNightWindow(QtWidgets.QMainWindow):
    """夜间模式：左侧坐标图，右侧雷达数据 + 跟踪步间隔下发 (0x59)。"""

    def __init__(self, vis: RadarVisualizer, raw_window: RawLogWindow, title: str):
        super().__init__()
        self.vis = vis
        self.raw_window = raw_window
        self._last_boundary_epoch = -1

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
            QtGui.QFont("Consolas", 11) if sys.platform == "win32" else QtGui.QFont("monospace", 11)
        )
        right_l.addWidget(self.radar_text, 1)

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
        self.speed_btn = QtWidgets.QPushButton("下发 CMD 0x59 (RADAR_TRACK_SPEED)")
        self.speed_btn.clicked.connect(self._on_send_speed)
        speed_l.addWidget(self.speed_btn)
        if vis._transport != "ble":
            self.speed_spin.setEnabled(False)
            self.speed_btn.setEnabled(False)
            self.speed_btn.setToolTip("仅 BLE 模式可下发")
        right_l.addWidget(speed_box)

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

    def _on_send_speed(self) -> None:
        ok = self.vis.send_radar_track_interval_us(self.speed_spin.value())
        if not ok:
            self.radar_text.appendPlainText(
                "[本地] 下发失败（非 BLE 或未连接）\n"
            )

    def _setup_plot(self):
        C_TEXT = "#cdd6f4"
        C_GRID = "#45475a"
        self.ax.tick_params(colors=C_TEXT)
        for s in self.ax.spines.values():
            s.set_color(C_GRID)
        self.ax.xaxis.label.set_color(C_TEXT)
        self.ax.yaxis.label.set_color(C_TEXT)
        self.ax.title.set_color("#89b4fa")

        with self.vis._lock:
            bq = [tuple(p) for p in self.vis.boundary_quad]
        quad = bq + [bq[0]]
        quad_xs = [p[0] for p in quad]
        quad_ys = [p[1] for p in quad]
        (self.boundary_line,) = self.ax.plot(
            quad_xs,
            quad_ys,
            color="#89b4fa",
            linewidth=2,
            alpha=0.95,
            label="Boundary",
        )

        (self.prev_point,) = self.ax.plot(
            [], [], marker="x", color="#f9e2af", markersize=9, mew=2, label="Prev"
        )
        (self.raw_point,) = self.ax.plot(
            [], [], "o", color="#89dceb", markersize=7, label="Raw"
        )
        (self.pred_a_point,) = self.ax.plot(
            [], [], "s", color="#a6e3a1", markersize=6, label="Pred A"
        )
        (self.pred_b_point,) = self.ax.plot(
            [], [], "s", color="#94e2d5", markersize=6, label="Pred B"
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
            if not self.vis.raw_lines:
                return
            lines = list(self.vis.raw_lines)
            self.vis.raw_lines.clear()
        self.raw_window.append_lines(lines)

    def _update_view(self):
        with self.vis._lock:
            latest_prev = self.vis.latest_prev
            latest_raw = self.vis.latest_raw
            latest_a = self.vis.latest_pred_a
            latest_b = self.vis.latest_pred_b
            seq = list(self.vis.seq_history)
            bepoch = self.vis.boundary_epoch
            bquad = [tuple(p) for p in self.vis.boundary_quad]
            mdir_ok = self.vis.motion_dir_valid
            mdeg10 = self.vis.motion_dir_deg10

        if bepoch != self._last_boundary_epoch:
            self._last_boundary_epoch = bepoch
            q = bquad + [bquad[0]]
            self.boundary_line.set_data([p[0] for p in q], [p[1] for p in q])

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

        if latest_a is not None:
            self.pred_a_point.set_data([latest_a[0]], [latest_a[1]])
        else:
            self.pred_a_point.set_data([], [])

        if latest_b is not None:
            self.pred_b_point.set_data([latest_b[0]], [latest_b[1]])
        else:
            self.pred_b_point.set_data([], [])

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
            f"Pred A:   {latest_a if latest_a else '—'}",
            f"Pred B:   {latest_b if latest_b else '—'}",
            f"Seq pts:  {len(seq)}  {seq[-3:] if seq else ''}",
            "",
            "── 场地 ──",
            f"Boundary epoch: {bepoch}",
            f"Quad: {bquad}",
            "",
            f"UI 待下发 interval: {self.speed_spin.value()} µs  (CMD 0x59)",
        ]
        self.radar_text.setPlainText("\n".join(lines))

        self.canvas.draw_idle()


# python visializable.py --transport ble --address 00:01:30:00:00:40
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

    if args.transport == "ble" and not args.address:
        print("BLE mode requires --address <MAC>", file=sys.stderr)
        sys.exit(2)

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
        raw_title = "Raw BLE notify (hex)"
        win_title = f"Radar visualizer (BLE {args.address})"
    else:
        raw_title = "Raw serial lines"
        win_title = f"Radar visualizer (serial {args.port})"

    raw_window = RawLogWindow(raw_title)
    raw_window.show()

    main_window = RadarNightWindow(vis, raw_window, win_title)
    main_window.show()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        vis.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
