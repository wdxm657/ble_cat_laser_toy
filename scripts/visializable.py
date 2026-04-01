import sys
import threading
import time
from collections import deque

import serial
from PyQt5 import QtCore, QtWidgets

import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


PORT = "COM3"
BAUDRATE = 115200

# x: [-1100, 1100], y: [100, 4100]
X_MIN, X_MAX = -1300, 1300
Y_MIN, Y_MAX = 100, 4100

BOUNDARY_QUAD = [
    (-1000, 800),
    (1000, 800),
    (1000, 4000),
    (-1000, 4000),
]


class RadarVisualizer:
    def __init__(self, port: str, baudrate: int):
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.seq_history = deque(maxlen=9)

        self.latest_prev = None
        self.latest_raw = None
        self.latest_pred_a = None
        self.latest_pred_b = None

        self.raw_lines = deque(maxlen=2000)

    def start(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()

    def close(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                with self._lock:
                    self.raw_lines.append(line)
                self._parse_line(line[8:])
            except Exception:
                time.sleep(0.01)

    def _parse_line(self, line: str):
        # MCU output protocol:
        # PREV,x,y
        # PREV,x,y,RAW,x,y
        # RAW,x,y,v   (legacy)
        # RAW,x,y     (optional)
        # PRED,STA,ax,ay,aang,bx,by,bang
        # PREDSEQ,idx,x,y,ang
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            return

        try:
            with self._lock:
                # Combined line: PREV,prevx,prevy,RAW,x,y
                if parts[0] == "PREV" and len(parts) >= 6 and parts[3] == "RAW":
                    prev_x = int(parts[1])
                    prev_y = int(parts[2])
                    raw_x = int(parts[4])
                    raw_y = int(parts[5])
                    self.latest_prev = (prev_x, prev_y)
                    self.latest_raw = (raw_x, raw_y, None)
                    self.latest_pred_a = None
                    self.latest_pred_b = None
                    self.seq_history.clear()
                elif parts[0] == "PRED" and parts[1] == "STA" and len(parts) >= 6:
                    ax = int(parts[2])
                    ay = int(parts[3])
                    bx = int(parts[4])
                    by = int(parts[5])
                    self.latest_pred_a = (ax, ay)
                    self.latest_pred_b = (bx, by)
                    self.seq_history.clear()
                elif parts[0] == "PREDSEQ" and len(parts) >= 4:
                    self.latest_pred_a = None
                    self.latest_pred_b = None
                    idx = int(parts[1])
                    if idx == 1:
                        self.seq_history.clear()
                    x = int(parts[2])
                    y = int(parts[3])
                    self.seq_history.append((x, y))
        except ValueError:
            # Ignore malformed lines
            return


class RawLogWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Raw Serial Data")
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
    def __init__(self, vis: RadarVisualizer, raw_window: RawLogWindow):
        super().__init__()
        self.vis = vis
        self.raw_window = raw_window

        self.setWindowTitle(f"Radar Raw/Prediction Visualizer ({PORT})")
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
        quad = BOUNDARY_QUAD + [BOUNDARY_QUAD[0]]
        quad_xs = [p[0] for p in quad]
        quad_ys = [p[1] for p in quad]
        (self.boundary_line,) = self.ax.plot(
            quad_xs, quad_ys, "k-", linewidth=2, alpha=0.8, label="Boundary (Quad)"
        )

        (self.prev_point,) = self.ax.plot([], [], "yx", markersize=8, mew=2, label="Prev")
        (self.raw_point,) = self.ax.plot([], [], "bo", label="Raw")
        (self.pred_a_point,) = self.ax.plot([], [], "go", label="Pred A (STA)")
        (self.pred_b_point,) = self.ax.plot([], [], "co", label="Pred B (STA)")
        (self.seq_points,) = self.ax.plot([], [], "r.", alpha=0.6, label="Pred Seq")

        self.raw_text = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, va="top", fontsize=10)

        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_xlim(X_MIN - 100, X_MAX + 100)
        self.ax.set_ylim(Y_MIN - 100, Y_MAX + 100)
        self.ax.grid(True, linestyle="--", alpha=0.4)
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

        if latest_prev is not None:
            self.prev_point.set_data([latest_prev[0]], [latest_prev[1]])
        else:
            self.prev_point.set_data([], [])

        if latest_raw is not None:
            x, y, v = latest_raw
            self.raw_point.set_data([x], [y])
            if v is None:
                self.raw_text.set_text(f"RAW x={x}mm y={y}mm")
            else:
                self.raw_text.set_text(f"RAW x={x}mm y={y}mm v={v}cm/s")
        else:
            self.raw_point.set_data([], [])
            self.raw_text.set_text("RAW: N/A")

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


def main():
    app = QtWidgets.QApplication(sys.argv)
    vis = RadarVisualizer(PORT, BAUDRATE)
    vis.start()

    raw_window = RawLogWindow()
    raw_window.show()

    main_window = RadarWindow(vis, raw_window)
    main_window.show()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        vis.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
