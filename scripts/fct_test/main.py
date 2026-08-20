import asyncio
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from bleak import BleakScanner
except ImportError:
    BleakScanner = None

from PyQt5 import QtCore, QtWidgets

try:
    from .protocol import (
        CMD_GPIO_SET,
        CMD_LOW_POWER,
        CMD_UID_READ,
        EVT_ADC,
        EVT_GPIO,
        EVT_KEY,
        EVT_UID,
        FrameParser,
        STATUS_TEXT,
        build_command,
    )
except ImportError:
    from protocol import (
        CMD_GPIO_SET,
        CMD_LOW_POWER,
        CMD_UID_READ,
        EVT_ADC,
        EVT_GPIO,
        EVT_KEY,
        EVT_UID,
        FrameParser,
        STATUS_TEXT,
        build_command,
    )


GPIO_NAMES = [
    "PC0", "PC1", "PC2", "PC3", "PB7",
    "PB6", "PB5", "PB4", "PC6", "PD7",
]


class SerialWorker(QtCore.QObject):
    frame = QtCore.pyqtSignal(dict)
    log = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.port = None
        self._thread = None
        self._stop = threading.Event()
        self._seq = 0
        self._parser = FrameParser()

    def open(self, port_name: str, baud: int):
        if serial is None:
            self.log.emit("缺少 pyserial，请安装 pyserial")
            return
        self.close()
        try:
            self.port = serial.Serial(port_name, baudrate=baud, timeout=0.1)
            self._stop.clear()
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.connected.emit(True)
            self.log.emit(f"串口已打开: {port_name}")
        except Exception as exc:
            self.log.emit(f"打开串口失败: {exc}")

    def close(self):
        self._stop.set()
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.connected.emit(False)

    def send(self, cmd_id: int, payload: bytes = b""):
        if self.port is None or not self.port.is_open:
            self.log.emit("串口未打开")
            return
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        try:
            frame = build_command(cmd_id, seq, payload)
            print(f"TX {len(frame)}B: {frame.hex(' ').upper()}", flush=True)
            self.port.write(frame)
        except Exception as exc:
            self.log.emit(f"串口发送失败: {exc}")

    def _read_loop(self):
        while not self._stop.is_set() and self.port is not None:
            try:
                data = self.port.read(128)
                if data:
                    print(f"RX {len(data)}B: {data.hex(' ').upper()}", flush=True)
                for frame in self._parser.feed(data):
                    self.frame.emit(frame)
            except Exception as exc:
                self.log.emit(f"串口读取失败: {exc}")
                break


class FctWindow(QtWidgets.QWidget):
    scan_finished = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.worker = SerialWorker()
        self.uid = b""
        self._build_ui()
        self.worker.frame.connect(self._on_frame)
        self.worker.log.connect(self._log)
        self.worker.connected.connect(self._on_connected)
        self.scan_finished.connect(self._finish_scan)
        self.refresh_ports()

    def _build_ui(self):
        self.setWindowTitle("B80 FCT 量产测试")
        self.resize(760, 620)
        root = QtWidgets.QVBoxLayout(self)

        port_row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("刷新串口")
        self.open_btn = QtWidgets.QPushButton("打开串口")
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "9600"])
        port_row.addWidget(QtWidgets.QLabel("串口"))
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(QtWidgets.QLabel("波特率"))
        port_row.addWidget(self.baud_combo)
        port_row.addWidget(self.refresh_btn)
        port_row.addWidget(self.open_btn)
        root.addLayout(port_row)

        sensor_box = QtWidgets.QGroupBox("实时状态")
        sensor_grid = QtWidgets.QGridLayout(sensor_box)
        self.bat_label = QtWidgets.QLabel("-- mV")
        self.ntc_label = QtWidgets.QLabel("-- mV")
        self.key_label = QtWidgets.QLabel("未知")
        self.uid_label = QtWidgets.QLabel("未读取")
        self.ble_label = self._result_label("默认")
        for row, (name, widget) in enumerate([
            ("电池 ADC", self.bat_label), ("NTC ADC", self.ntc_label),
            ("按键", self.key_label), ("Flash UID", self.uid_label),
            ("BLE UID 匹配", self.ble_label),
        ]):
            sensor_grid.addWidget(QtWidgets.QLabel(name), row // 2, (row % 2) * 2)
            sensor_grid.addWidget(widget, row // 2, (row % 2) * 2 + 1)
        root.addWidget(sensor_box)

        action_row = QtWidgets.QHBoxLayout()
        self.uid_btn = QtWidgets.QPushButton("读取 UID")
        self.scan_btn = QtWidgets.QPushButton("扫描 BLE 3 秒")
        self.sleep_btn = QtWidgets.QPushButton("进入低功耗")
        action_row.addWidget(self.uid_btn)
        action_row.addWidget(self.scan_btn)
        action_row.addWidget(self.sleep_btn)
        root.addLayout(action_row)

        gpio_box = QtWidgets.QGroupBox("GPIO 控制")
        gpio_grid = QtWidgets.QGridLayout(gpio_box)
        for index, name in enumerate(GPIO_NAMES):
            button = QtWidgets.QPushButton(f"{name} 开")
            button.setCheckable(True)
            button.setProperty("gpio_index", index)
            button.clicked.connect(self._gpio_clicked)
            gpio_grid.addWidget(button, index // 5, index % 5)
        root.addWidget(gpio_box)

        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        root.addWidget(self.log_edit, 1)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.open_btn.clicked.connect(self._toggle_serial)
        self.uid_btn.clicked.connect(lambda: self.worker.send(CMD_UID_READ))
        self.scan_btn.clicked.connect(self._scan_ble)
        self.sleep_btn.clicked.connect(lambda: self.worker.send(CMD_LOW_POWER))

    def _result_label(self, text):
        label = QtWidgets.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMinimumWidth(100)
        self._set_result(label, "default")
        return label

    def _set_result(self, label, state):
        colors = {"default": "#9ca3af", "ok": "#16a34a", "fail": "#dc2626"}
        label.setStyleSheet(f"color: white; background: {colors[state]}; padding: 4px;")

    def refresh_ports(self):
        self.port_combo.clear()
        if list_ports is not None:
            self.port_combo.addItems([p.device for p in list_ports.comports()])

    def _toggle_serial(self):
        if self.worker.port is not None:
            self.worker.close()
            self.open_btn.setText("打开串口")
        else:
            self.worker.open(self.port_combo.currentText(), int(self.baud_combo.currentText()))

    def _on_connected(self, connected):
        self.open_btn.setText("关闭串口" if connected else "打开串口")

    def _gpio_clicked(self):
        button = self.sender()
        index = int(button.property("gpio_index"))
        level = 1 if button.isChecked() else 0
        button.setText(f"{GPIO_NAMES[index]} {'关' if level == 0 else '开'}")
        self.worker.send(CMD_GPIO_SET, bytes([index, level]))

    def _on_frame(self, frame):
        payload = frame["payload"]
        if frame["type"] == 0x02 and payload:
            self._log(f"响应 cmd=0x{frame['cmd']:02X}: {STATUS_TEXT.get(payload[0], hex(payload[0]))}")
            if frame["cmd"] == CMD_UID_READ and len(payload) >= 17 and payload[0] == 0:
                self.uid = bytes(payload[1:17])
                self.uid_label.setText(self.uid.hex().upper())
                self._log(f"UID: {self.uid.hex().upper()}")
        elif frame["type"] != 0x03:
            return
        elif frame["cmd"] == EVT_ADC and len(payload) >= 4:
            self.bat_label.setText(f"{int.from_bytes(payload[0:2], 'little')} mV")
            self.ntc_label.setText(f"{int.from_bytes(payload[2:4], 'little')} mV")
            if len(payload) >= 5:
                self.key_label.setText("按下" if payload[4] else "松开")
        elif frame["cmd"] == EVT_KEY and payload:
            self.key_label.setText("按下" if payload[0] else "松开")
        elif frame["cmd"] == EVT_UID:
            self.uid = bytes(payload)
            self.uid_label.setText(self.uid.hex().upper())
            self._log(f"UID: {self.uid.hex().upper()}")
        elif frame["cmd"] == EVT_GPIO and len(payload) >= 2:
            self._log(f"GPIO {GPIO_NAMES[payload[0]] if payload[0] < len(GPIO_NAMES) else payload[0]} = {payload[1]}")

    def _scan_ble(self):
        if BleakScanner is None:
            self._log("缺少 bleak，请安装 bleak")
            return
        if not self.uid:
            self._log("请先读取 UID")
            return
        self.scan_btn.setEnabled(False)
        threading.Thread(target=self._scan_ble_thread, daemon=True).start()

    def _scan_ble_thread(self):
        found = False
        try:
            async def scan():
                nonlocal found
                devices = await BleakScanner.discover(timeout=3.0, return_adv=True)
                target = self.uid.hex().lower()
                for device, adv in devices.values():
                    values = []
                    for data in (getattr(adv, "manufacturer_data", {}) or {}).values():
                        values.append(bytes(data).hex().lower())
                    if target in "".join(values) or target[::-1] in "".join(values):
                        found = True
                        break
            asyncio.run(scan())
        except Exception as exc:
            self.worker.log.emit(f"BLE 扫描失败: {exc}")
        self.scan_finished.emit(found)

    @QtCore.pyqtSlot(bool)
    def _finish_scan(self, found):
        self.ble_label.setText("匹配" if found else "不匹配")
        self._set_result(self.ble_label, "ok" if found else "fail")
        self.scan_btn.setEnabled(True)
        self._log("BLE UID 匹配成功" if found else "BLE UID 未匹配")

    def _log(self, text):
        self.log_edit.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")

    def closeEvent(self, event):
        self.worker.close()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = FctWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
