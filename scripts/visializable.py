"""
Radar prediction debug visualizer.

- Serial: text lines after 8-char log prefix, e.g. PREV,...,RAW,... / PRED,STA,... / PREDSEQ,...
- BLE: binary EVENT on Ctrl TX (cmd 0x57): prediction debug + BOUNDARY_PT (0x04).
  BOUNDARY_PT is **not** streamed: device sends 4 NOTIFY only after CMD **0x58** on Ctrl RX.
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
import struct
import sys
import threading
import time
import traceback
import uuid
from collections import deque
from typing import List, Optional, Set, Tuple

import serial
from PyQt5 import QtCore, QtWidgets

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
                    print(f"[BLE] notify on {ctrl_tx.uuid}", file=sys.stderr)
                    await client.start_notify(ctrl_tx, on_notify)
                    if self._ble_request_boundary:

                        async def _write_get_boundary(rx_ch) -> None:
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
                                await client.write_gatt_char(rx_ch, req, response=True)
                            except Exception:
                                await client.write_gatt_char(rx_ch, req, response=False)

                        await asyncio.sleep(0.25)
                        try:
                            rx_ch = _find_ctrl_rx_characteristic(client)
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


class RawLogWindow(QtWidgets.QMainWindow):
    def __init__(self, title: str = "Raw data"):
        super().__init__()
        self.setWindowTitle(title)
        self.resize(520, 600)
        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.setCentralWidget(self.text)

    def append_lines(self, lines):
        if not lines:
            return
        self.text.appendPlainText("\n".join(lines))
        self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())


class RadarWindow(QtWidgets.QMainWindow):
    def __init__(self, vis: RadarVisualizer, raw_window: RawLogWindow, title: str):
        super().__init__()
        self.vis = vis
        self.raw_window = raw_window
        self._last_boundary_epoch = -1

        self.setWindowTitle(title)
        self.resize(760, 860)

        self.figure = Figure(figsize=(7, 9), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.setCentralWidget(self.canvas)

        self.ax = self.figure.add_subplot(111)
        self._setup_plot()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(80)
        self.timer.timeout.connect(self._update_view)
        self.timer.start()

        self.log_timer = QtCore.QTimer(self)
        self.log_timer.setInterval(60)
        self.log_timer.timeout.connect(self._flush_raw_log)
        self.log_timer.start()

    def _setup_plot(self):
        with self.vis._lock:
            bq = [tuple(p) for p in self.vis.boundary_quad]
        quad = bq + [bq[0]]
        quad_xs = [p[0] for p in quad]
        quad_ys = [p[1] for p in quad]
        (self.boundary_line,) = self.ax.plot(
            quad_xs, quad_ys, "k-", linewidth=2, alpha=0.8, label="Boundary (Quad)"
        )

        (self.prev_point,) = self.ax.plot(
            [], [], "yx", markersize=8, mew=2, label="Prev"
        )
        (self.raw_point,) = self.ax.plot([], [], "bo", label="Raw")
        (self.pred_a_point,) = self.ax.plot([], [], "go", label="Pred A (STA)")
        (self.pred_b_point,) = self.ax.plot([], [], "co", label="Pred B (STA)")
        (self.seq_points,) = self.ax.plot([], [], "r.", alpha=0.6, label="Pred Seq")

        self.raw_text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes, va="top", fontsize=10
        )

        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_xlim(X_MIN - 100, X_MAX + 100)
        self.ax.set_ylim(Y_MIN - 100, Y_MAX + 100)
        self.ax.grid(True, linestyle="--", alpha=0.4)

        self.motion_arrow = FancyArrowPatch(
            (0.0, 0.0),
            (0.0, 0.0),
            arrowstyle="-|>",
            mutation_scale=32,
            linewidth=4.0,
            edgecolor="#FF0099",
            facecolor="#FF0099",
            zorder=9,
            label="Motion dir (1s win)",
        )
        self.ax.add_patch(self.motion_arrow)
        self.motion_arrow.set_visible(False)

        self.ax.legend(loc="lower right")
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
                self.raw_text.set_text(f"RAW x={x}mm y={y}mm  θ={th:.1f}° (1s)")
            elif v is None:
                self.raw_text.set_text(f"RAW x={x}mm y={y}mm")
            else:
                self.raw_text.set_text(f"RAW x={x}mm y={y}mm v={v}cm/s")
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
            self.raw_text.set_text("RAW: N/A")
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

    main_window = RadarWindow(vis, raw_window, win_title)
    main_window.show()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        vis.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
