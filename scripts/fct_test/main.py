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
        CMD_GPIO_ALL_SET,
        CMD_BAT_ADC_READ,
        CMD_FW_VERSION_READ,
        CMD_LOW_POWER,
        CMD_NTC_ADC_READ,
        CMD_UID_READ,
        EVT_ADC,
        EVT_GPIO,
        EVT_KEY,
        EVT_UID,
        EVT_USB,
        FrameParser,
        STATUS_TEXT,
        build_command,
    )
except ImportError:
    from protocol import (
        CMD_GPIO_SET,
        CMD_GPIO_ALL_SET,
        CMD_BAT_ADC_READ,
        CMD_FW_VERSION_READ,
        CMD_LOW_POWER,
        CMD_NTC_ADC_READ,
        CMD_UID_READ,
        EVT_ADC,
        EVT_GPIO,
        EVT_KEY,
        EVT_UID,
        EVT_USB,
        FrameParser,
        STATUS_TEXT,
        build_command,
    )


GPIO_NAMES = [
    " ", " ", " ", " ", " ",
    " ", " ", " ", " ", " ",
]
GPIO_ALIASES = [
    "1A", "1B", "1C", "1D", "2D",
    "2C", "2B", "2A", "Laser", "5v+",
]
GPIO_LABELS = [f"{pin} ({alias})" for pin, alias in zip(GPIO_NAMES, GPIO_ALIASES)]
BLE_DEVICE_NAME = "W2MLaserTOY"


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
    ble_devices_updated = QtCore.pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.worker = SerialWorker()
        self.uid = b""
        self._ble_stop = threading.Event()
        self._ble_devices = {}
        self._build_ui()
        self.worker.frame.connect(self._on_frame)
        self.worker.log.connect(self._log)
        self.worker.connected.connect(self._on_connected)
        self.ble_devices_updated.connect(self._on_ble_devices_updated)
        self.refresh_ports()
        self._start_ble_scan()

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
        self.usb_label = QtWidgets.QLabel("未知")
        self.uid_label = QtWidgets.QLabel("未读取")
        self.fw_version_label = QtWidgets.QLabel("未读取")
        self.ble_label = self._result_label("默认")
        for row, (name, widget) in enumerate([
            ("电池电压", self.bat_label), ("NTC 电压", self.ntc_label),
            ("按键", self.key_label), ("USB", self.usb_label),
            ("Flash UID", self.uid_label), ("固件版本", self.fw_version_label),
            ("BLE UID 匹配", self.ble_label),
        ]):
            sensor_grid.addWidget(QtWidgets.QLabel(name), row // 2, (row % 2) * 2)
            sensor_grid.addWidget(widget, row // 2, (row % 2) * 2 + 1)
        root.addWidget(sensor_box)

        action_row = QtWidgets.QHBoxLayout()
        self.bat_btn = QtWidgets.QPushButton("读取电池 ADC")
        self.ntc_btn = QtWidgets.QPushButton("读取 NTC ADC")
        self.uid_btn = QtWidgets.QPushButton("读取 UID")
        self.fw_version_btn = QtWidgets.QPushButton("读取固件版本")
        self.sleep_btn = QtWidgets.QPushButton("进入低功耗")
        action_row.addWidget(self.bat_btn)
        action_row.addWidget(self.ntc_btn)
        action_row.addWidget(self.uid_btn)
        action_row.addWidget(self.fw_version_btn)
        action_row.addWidget(self.sleep_btn)
        root.addLayout(action_row)

        self.tabs = QtWidgets.QTabWidget()
        self.fct_page = QtWidgets.QWidget()
        self.ble_page = QtWidgets.QWidget()
        fct_layout = QtWidgets.QVBoxLayout(self.fct_page)
        ble_layout = QtWidgets.QVBoxLayout(self.ble_page)

        gpio_box = QtWidgets.QGroupBox("GPIO 控制")
        gpio_grid = QtWidgets.QGridLayout(gpio_box)
        self.all_gpio_btn = QtWidgets.QPushButton("全部 GPIO 开")
        self.all_gpio_btn.setCheckable(True)
        self.all_gpio_btn.setProperty("all_gpio_control", True)
        self.all_gpio_btn.clicked.connect(self._all_gpio_clicked)
        gpio_grid.addWidget(self.all_gpio_btn, 0, 0, 1, 5)
        self._gpio_buttons = []
        for index, name in enumerate(GPIO_NAMES):
            button = QtWidgets.QPushButton(f"{GPIO_LABELS[index]} 开")
            button.setCheckable(True)
            button.setProperty("gpio_index", index)
            button.clicked.connect(self._gpio_clicked)
            self._gpio_buttons.append(button)
            gpio_grid.addWidget(button, index // 5 + 1, index % 5)
        fct_layout.addWidget(gpio_box)

        self.ble_table = QtWidgets.QTableWidget(0, 4)
        self.ble_table.setHorizontalHeaderLabels(["匹配", "地址", "名称", "Manufacturer Specific Data"])
        self.ble_table.horizontalHeader().setStretchLastSection(True)
        self.ble_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.ble_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        ble_layout.addWidget(self.ble_table)

        self.tabs.addTab(self.fct_page, "FCT 控制")
        self.tabs.addTab(self.ble_page, "BLE 设备")
        root.addWidget(self.tabs)

        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        root.addWidget(self.log_edit, 1)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.open_btn.clicked.connect(self._toggle_serial)
        self.bat_btn.clicked.connect(lambda: self.worker.send(CMD_BAT_ADC_READ))
        self.ntc_btn.clicked.connect(lambda: self.worker.send(CMD_NTC_ADC_READ))
        self.uid_btn.clicked.connect(lambda: self.worker.send(CMD_UID_READ))
        self.fw_version_btn.clicked.connect(lambda: self.worker.send(CMD_FW_VERSION_READ))
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
        button.setText(f"{GPIO_LABELS[index]} {'关' if level == 0 else '开'}")
        self.worker.send(CMD_GPIO_SET, bytes([index, level]))
        self._sync_all_gpio_button()

    def _all_gpio_clicked(self):
        level = 1 if self.all_gpio_btn.isChecked() else 0
        self.all_gpio_btn.setText(f"全部 GPIO {'关' if level == 0 else '开'}")
        for index, name in enumerate(GPIO_NAMES):
            button = self._gpio_buttons[index]
            button.blockSignals(True)
            button.setChecked(bool(level))
            button.setText(f"{GPIO_LABELS[index]} {'关' if level == 0 else '开'}")
            button.blockSignals(False)
        self.worker.send(CMD_GPIO_ALL_SET, bytes([level]))

    def _sync_all_gpio_button(self):
        all_on = all(button.isChecked() for button in self._gpio_buttons)
        self.all_gpio_btn.blockSignals(True)
        self.all_gpio_btn.setChecked(all_on)
        self.all_gpio_btn.setText(f"全部 GPIO {'开' if all_on else '关'}")
        self.all_gpio_btn.blockSignals(False)

    def _on_frame(self, frame):
        payload = frame["payload"]
        if frame["type"] == 0x02 and payload:
            self._log(f"响应 cmd=0x{frame['cmd']:02X}: {STATUS_TEXT.get(payload[0], hex(payload[0]))}")
            if frame["cmd"] in (CMD_BAT_ADC_READ, CMD_NTC_ADC_READ) and len(payload) >= 3 and payload[0] == 0:
                value = int.from_bytes(payload[1:3], "little")
                if frame["cmd"] == CMD_BAT_ADC_READ:
                    self.bat_label.setText(f"{value} mV")
                    self._log(f"电池 ADC: {value} mV")
                else:
                    self.ntc_label.setText(f"{value} mV")
                    self._log(f"NTC ADC: {value} mV")
            elif frame["cmd"] == CMD_UID_READ and len(payload) >= 17 and payload[0] == 0:
                self.uid = bytes(payload[1:17])
                self.uid_label.setText(self.uid.hex().upper())
                self._log(f"UID: {self.uid.hex().upper()}")
                self._refresh_ble_table()
            elif frame["cmd"] == CMD_FW_VERSION_READ and len(payload) >= 4 and payload[0] == 0:
                version = f"{payload[1]}.{payload[2]}.{payload[3]}"
                self.fw_version_label.setText(version)
                self._log(f"固件版本: {version}")
        elif frame["type"] != 0x03:
            return
        elif frame["cmd"] == EVT_ADC and len(payload) >= 4:
            self.bat_label.setText(f"{int.from_bytes(payload[0:2], 'little')} mV")
            self.ntc_label.setText(f"{int.from_bytes(payload[2:4], 'little')} mV")
            if len(payload) >= 5:
                self.key_label.setText("按下" if payload[4] else "松开")
        elif frame["cmd"] == EVT_KEY and payload:
            self.key_label.setText("按下" if payload[0] else "松开")
        elif frame["cmd"] == EVT_USB and payload:
            self.usb_label.setText("已插入" if payload[0] else "已拔出")
        elif frame["cmd"] == EVT_UID:
            self.uid = bytes(payload)
            self.uid_label.setText(self.uid.hex().upper())
            self._log(f"UID: {self.uid.hex().upper()}")
            self._refresh_ble_table()
        elif frame["cmd"] == EVT_GPIO and len(payload) >= 2:
            if payload[0] < len(GPIO_LABELS):
                self._log(f"GPIO {GPIO_LABELS[payload[0]]} = {payload[1]}")
            else:
                self._log(f"GPIO {payload[0]} = {payload[1]}")

    def _start_ble_scan(self):
        if BleakScanner is None:
            self._log("缺少 bleak，请安装 bleak")
            return
        self._ble_stop.clear()
        threading.Thread(target=self._ble_scan_loop, daemon=True).start()

    def _ble_scan_loop(self):
        while not self._ble_stop.is_set():
            try:
                devices_info = asyncio.run(self._discover_ble_devices())
            except Exception as exc:
                self.worker.log.emit(f"BLE 扫描失败: {exc}")
                devices_info = []

            self.ble_devices_updated.emit(devices_info)

    async def _discover_ble_devices(self):
        devices_info = []
        devices = await BleakScanner.discover(timeout=3.0, return_adv=True)
        for device, adv in devices.values():
            device_name = getattr(device, "name", None)
            adv_name = getattr(adv, "local_name", None)
            if device_name != BLE_DEVICE_NAME and adv_name != BLE_DEVICE_NAME:
                continue

            manufacturer_data = getattr(adv, "manufacturer_data", {}) or {}
            manufacturer_items = []
            manufacturer_raw = []
            for company_id, data in manufacturer_data.items():
                raw_data = company_id.to_bytes(2, "little") + bytes(data)
                manufacturer_raw.append(raw_data)
                manufacturer_items.append(raw_data.hex(" ").upper())

            address = getattr(device, "address", "")
            devices_info.append({
                "address": address,
                "name": adv_name or device_name or BLE_DEVICE_NAME,
                "manufacturer_text": " ".join(manufacturer_items),
                "manufacturer_raw": manufacturer_raw,
                "last_seen": time.time(),
            })
        return devices_info

    def _on_ble_devices_updated(self, devices_info):
        for item in devices_info:
            self._ble_devices[item["address"]] = item
        self._refresh_ble_table()

    def _ble_item_matches_uid(self, item):
        if not self.uid:
            return False
        target_uid = bytes(self.uid)
        return any(target_uid in raw for raw in item["manufacturer_raw"])

    def _refresh_ble_table(self):
        devices = sorted(self._ble_devices.values(), key=lambda item: item["address"])
        self.ble_table.setRowCount(len(devices))

        found = False
        for row, item in enumerate(devices):
            matched = self._ble_item_matches_uid(item)
            found = found or matched
            values = [
                "匹配" if matched else "",
                item["address"],
                item["name"],
                item["manufacturer_text"],
            ]
            for column, value in enumerate(values):
                table_item = QtWidgets.QTableWidgetItem(value)
                if matched:
                    table_item.setBackground(QtCore.Qt.green)
                self.ble_table.setItem(row, column, table_item)

        if self.uid:
            self.ble_label.setText("匹配" if found else "不匹配")
            self._set_result(self.ble_label, "ok" if found else "fail")
        else:
            self.ble_label.setText("默认")
            self._set_result(self.ble_label, "default")

    def _log(self, text):
        self.log_edit.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")

    def closeEvent(self, event):
        self._ble_stop.set()
        self.worker.close()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = FctWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
