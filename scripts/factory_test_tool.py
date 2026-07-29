# -*- coding: utf-8 -*-
"""
Assembly factory BLE test tool for W2M laser toy devices.

Dependencies:
    pip install PyQt5 bleak openpyxl

This tool intentionally does not auto-connect or auto-reconnect. The operator
scans, selects a device, connects manually, then sends the test or reboot
command.
"""

import asyncio
import datetime as _dt
import os
import sys
import threading
import uuid
from dataclasses import dataclass
from typing import Dict, Optional, Set

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt

try:
    from bleak import BleakClient, BleakScanner
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    BleakClient = None
    BleakScanner = None
    BLEAK_IMPORT_ERROR = ex
else:
    BLEAK_IMPORT_ERROR = None

try:
    from openpyxl import Workbook, load_workbook
except Exception as ex:  # pragma: no cover - shown in UI at runtime
    Workbook = None
    load_workbook = None
    OPENPYXL_IMPORT_ERROR = ex
else:
    OPENPYXL_IMPORT_ERROR = None


CTRL_PROTO_VERSION = 0x01
CTRL_MSG_TYPE_CMD = 0x01
CTRL_MSG_TYPE_RSP = 0x02
CTRL_MSG_TYPE_EVENT = 0x03

CTRL_STATUS_TEXT = {
    0x00: "OK",
    0x01: "LEN_ERROR",
    0x02: "UNSUPPORTED_CMD",
    0x03: "PARAM_ERROR",
    0x04: "INTERNAL_ERROR",
    0x05: "REJECT_ERROR",
}

CTRL_CMD_DEVICE_REBOOT = 0x5A
CTRL_CMD_FACTORY_TEST_ENTER = 0x5D

EXCEL_HEADERS = [
    "时间",
    "设备名称",
    "MAC/地址",
    "MANUFACTURER_DATA",
    "雷达测试",
    "电机测试",
    "激光灯测试",
    "结果",
]

CTRL_RX_RAW_BYTES = bytes(
    [0x01, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_TX_RAW_BYTES = bytes(
    [0x02, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_LOG_RAW_BYTES = bytes(
    [0x03, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)


def _uuid_candidates(raw: bytes) -> Set[int]:
    return {uuid.UUID(bytes=raw).int, uuid.UUID(bytes=raw[::-1]).int}


def _char_uuid_int(char) -> Optional[int]:
    try:
        return uuid.UUID(str(char.uuid)).int
    except Exception:
        return None


def _find_characteristic(client, raw_uuid: bytes):
    targets = _uuid_candidates(raw_uuid)
    for service in client.services:
        for char in service.characteristics:
            if _char_uuid_int(char) in targets:
                return char
    return None


def _format_manufacturer_data(mfr: Dict[int, bytes]) -> str:
    if not mfr:
        return ""
    parts = []
    for company_id, data in sorted(mfr.items()):
        # Bleak exposes the first two manufacturer bytes as a little-endian
        # company_id. Rebuild the original advertising payload byte order.
        raw = int(company_id).to_bytes(2, "little") + bytes(data)
        parts.append("0x" + raw.hex().upper())
    return "; ".join(parts)


def _build_ctrl_cmd(cmd_id: int, seq: int, payload: bytes = b"") -> bytes:
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


def _parse_ctrl_frame(data: bytes) -> str:
    if len(data) < 6:
        return f"CTRL short: {data.hex(' ').upper()}"
    version, msg_type, cmd_id, seq = data[0], data[1], data[2], data[3]
    payload_len = data[4] | (data[5] << 8)
    payload = data[6 : 6 + payload_len]
    type_name = {CTRL_MSG_TYPE_CMD: "CMD", CTRL_MSG_TYPE_RSP: "RSP", CTRL_MSG_TYPE_EVENT: "EVT"}.get(msg_type, f"0x{msg_type:02X}")
    suffix = ""
    if msg_type == CTRL_MSG_TYPE_RSP and payload:
        suffix = f" status={CTRL_STATUS_TEXT.get(payload[0], f'0x{payload[0]:02X}')}"
    return f"[{type_name}] cmd=0x{cmd_id:02X} seq={seq} len={payload_len}{suffix} payload={payload.hex(' ').upper()}"


@dataclass
class ScanDevice:
    name: str
    address: str
    rssi: Optional[int]
    manufacturer_data: str


class BleWorker(QtCore.QObject):
    devices_changed = QtCore.pyqtSignal(list)
    log_line = QtCore.pyqtSignal(str)
    status_changed = QtCore.pyqtSignal(str, bool)

    def __init__(self):
        super().__init__()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._client = None
        self._rx_char = None
        self._seq = 0
        self._log_buf = bytearray()
        self._devices: Dict[str, ScanDevice] = {}

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def stop(self):
        try:
            self.disconnect()
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)

    def _schedule(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def scan(self, timeout_s: float = 5.0):
        self._schedule(self._scan(timeout_s))

    def connect(self, address: str):
        self._schedule(self._connect(address))

    def disconnect(self):
        self._schedule(self._disconnect())

    def send_factory_test_enter(self):
        self._schedule(self._send_cmd(CTRL_CMD_FACTORY_TEST_ENTER, "开始测试"))

    def send_reboot(self):
        self._schedule(self._send_cmd(CTRL_CMD_DEVICE_REBOOT, "重启设备"))

    async def _scan(self, timeout_s: float):
        if BLEAK_IMPORT_ERROR is not None:
            self.log_line.emit(f"Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}")
            return

        self.log_line.emit(f"开始扫描 W2M* 设备，{timeout_s:.0f}s...")
        found: Dict[str, ScanDevice] = {}

        def on_adv(device, adv):
            name = adv.local_name or getattr(device, "name", None) or ""
            if not name.startswith("W2M"):
                return
            rssi = getattr(adv, "rssi", None)
            if rssi is None:
                rssi = getattr(device, "rssi", None)
            found[device.address] = ScanDevice(
                name=name,
                address=device.address,
                rssi=rssi,
                manufacturer_data=_format_manufacturer_data(getattr(adv, "manufacturer_data", {}) or {}),
            )
            self._devices = dict(found)
            self.devices_changed.emit(list(found.values()))

        scanner = BleakScanner(detection_callback=on_adv)
        try:
            await scanner.start()
            await asyncio.sleep(timeout_s)
            await scanner.stop()
        except Exception as ex:
            self.log_line.emit(f"扫描失败: {ex}")
            return

        self._devices = dict(found)
        self.devices_changed.emit(list(found.values()))
        self.log_line.emit(f"扫描完成，找到 {len(found)} 个 W2M* 设备")

    async def _connect(self, address: str):
        if BLEAK_IMPORT_ERROR is not None:
            self.log_line.emit(f"Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}")
            return
        if not address:
            self.log_line.emit("请先选择设备")
            return
        if self._client and self._client.is_connected:
            self.log_line.emit("已连接，请先断开")
            return

        self.log_line.emit(f"连接中: {address}")

        def on_disconnect(_client):
            self._client = None
            self._rx_char = None
            self.status_changed.emit("已断开", False)
            self.log_line.emit("设备已断开")

        client = BleakClient(address, disconnected_callback=on_disconnect)
        try:
            await client.connect()
            get_services = getattr(client, "get_services", None)
            if get_services:
                out = get_services()
                if asyncio.iscoroutine(out):
                    await out
            else:
                _ = client.services

            self._rx_char = _find_characteristic(client, CTRL_RX_RAW_BYTES)
            tx_char = _find_characteristic(client, CTRL_TX_RAW_BYTES)
            log_char = _find_characteristic(client, CTRL_LOG_RAW_BYTES)
            if self._rx_char is None:
                raise RuntimeError("未找到 Ctrl RX 写特征")
            if tx_char is not None:
                await client.start_notify(tx_char, self._on_ctrl_notify)
            if log_char is not None:
                await client.start_notify(log_char, self._on_log_notify)
            else:
                self.log_line.emit("未找到 BLE_LOG_D 日志特征，仅显示控制响应")

            self._client = client
            self.status_changed.emit(f"已连接: {address}", True)
            self.log_line.emit("连接成功")
        except Exception as ex:
            try:
                await client.disconnect()
            except Exception:
                pass
            self._client = None
            self._rx_char = None
            self.status_changed.emit("连接失败", False)
            self.log_line.emit(f"连接失败: {ex}")

    async def _disconnect(self):
        client = self._client
        self._client = None
        self._rx_char = None
        if client is None:
            self.status_changed.emit("未连接", False)
            return
        try:
            if client.is_connected:
                await client.disconnect()
        except Exception as ex:
            self.log_line.emit(f"断开失败: {ex}")
            return
        self.status_changed.emit("已断开", False)
        self.log_line.emit("已手动断开")

    async def _send_cmd(self, cmd_id: int, label: str):
        client = self._client
        if client is None or not client.is_connected or self._rx_char is None:
            self.log_line.emit("未连接，无法发送命令")
            return
        seq = self._seq & 0xFF
        self._seq = (self._seq + 1) & 0xFF
        frame = _build_ctrl_cmd(cmd_id, seq)
        try:
            await client.write_gatt_char(self._rx_char, frame, response=False)
            self.log_line.emit(f"已发送 {label}: cmd=0x{cmd_id:02X} seq={seq}")
        except Exception as ex:
            self.log_line.emit(f"发送失败 {label}: {ex}")

    def _on_ctrl_notify(self, _sender, data: bytearray):
        self.log_line.emit(_parse_ctrl_frame(bytes(data)))

    def _on_log_notify(self, _sender, data: bytearray):
        self._log_buf.extend(bytes(data))
        while b"\n" in self._log_buf:
            idx = self._log_buf.index(0x0A)
            raw = bytes(self._log_buf[:idx]).rstrip(b"\r")
            del self._log_buf[: idx + 1]
            if raw:
                self.log_line.emit(raw.decode("utf-8", errors="replace"))
        if len(self._log_buf) > 512:
            raw = bytes(self._log_buf)
            self._log_buf.clear()
            self.log_line.emit(raw.decode("utf-8", errors="replace"))


class FactoryTestWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = BleWorker()
        self.devices: Dict[str, ScanDevice] = {}

        self.setWindowTitle("W2M 组装工厂测试工具")
        self.resize(960, 700)
        self._build_ui()
        self._connect_signals()
        self._set_connected(False)

        if BLEAK_IMPORT_ERROR is not None:
            self._append_log(f"Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}")

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        scan_row = QtWidgets.QHBoxLayout()
        self.scan_btn = QtWidgets.QPushButton("扫描 W2M 设备")
        self.device_combo = QtWidgets.QComboBox()
        self.device_combo.setMinimumWidth(360)
        self.connect_btn = QtWidgets.QPushButton("连接")
        self.disconnect_btn = QtWidgets.QPushButton("断开")
        scan_row.addWidget(self.scan_btn)
        scan_row.addWidget(self.device_combo, 1)
        scan_row.addWidget(self.connect_btn)
        scan_row.addWidget(self.disconnect_btn)
        root.addLayout(scan_row)

        self.status_label = QtWidgets.QLabel("未连接")
        root.addWidget(self.status_label)

        self.device_tree = QtWidgets.QTreeWidget()
        self.device_tree.setHeaderLabels(["名称", "MAC/地址", "RSSI", "MANUFACTURER_DATA"])
        self.device_tree.setRootIsDecorated(False)
        self.device_tree.setAlternatingRowColors(True)
        self.device_tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        root.addWidget(self.device_tree, 2)

        action_row = QtWidgets.QHBoxLayout()
        self.start_test_btn = QtWidgets.QPushButton("开始测试")
        self.reboot_btn = QtWidgets.QPushButton("重启设备")
        self.clear_log_btn = QtWidgets.QPushButton("清空日志")
        self.start_test_btn.setMinimumHeight(40)
        self.reboot_btn.setMinimumHeight(40)
        action_row.addWidget(self.start_test_btn)
        action_row.addWidget(self.reboot_btn)
        action_row.addStretch(1)
        action_row.addWidget(self.clear_log_btn)
        root.addLayout(action_row)

        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setPlaceholderText("BLE_LOG_D 日志和控制响应会显示在这里")
        root.addWidget(self.log_text, 3)

    def _connect_signals(self):
        self.scan_btn.clicked.connect(lambda: self.worker.scan())
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.disconnect_btn.clicked.connect(self.worker.disconnect)
        self.start_test_btn.clicked.connect(self.worker.send_factory_test_enter)
        self.reboot_btn.clicked.connect(self.worker.send_reboot)
        self.clear_log_btn.clicked.connect(self.log_text.clear)
        self.device_tree.itemDoubleClicked.connect(self._on_device_double_clicked)

        self.worker.devices_changed.connect(self._on_devices_changed)
        self.worker.log_line.connect(self._append_log)
        self.worker.status_changed.connect(self._on_status_changed)

    def _on_connect_clicked(self):
        address = self.device_combo.currentData()
        self.worker.connect(address or "")

    def _on_device_double_clicked(self, item, _column):
        address = item.data(0, Qt.UserRole)
        idx = self.device_combo.findData(address)
        if idx >= 0:
            self.device_combo.setCurrentIndex(idx)

    def _on_devices_changed(self, devices):
        current = self.device_combo.currentData()
        self.devices = {d.address: d for d in devices}

        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        for dev in devices:
            label = f"{dev.name}  {dev.address}"
            self.device_combo.addItem(label, dev.address)
        if current:
            idx = self.device_combo.findData(current)
            if idx >= 0:
                self.device_combo.setCurrentIndex(idx)
        self.device_combo.blockSignals(False)

        self.device_tree.clear()
        for dev in devices:
            item = QtWidgets.QTreeWidgetItem(
                [
                    dev.name,
                    dev.address,
                    "" if dev.rssi is None else str(dev.rssi),
                    dev.manufacturer_data,
                ]
            )
            item.setData(0, Qt.UserRole, dev.address)
            self.device_tree.addTopLevelItem(item)
        if self.device_combo.count() > 0 and self.device_combo.currentIndex() < 0:
            self.device_combo.setCurrentIndex(0)

    def _on_status_changed(self, text: str, connected: bool):
        self.status_label.setText(text)
        self._set_connected(connected)

    def _set_connected(self, connected: bool):
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.start_test_btn.setEnabled(connected)
        self.reboot_btn.setEnabled(connected)

    def _append_log(self, line: str):
        now = _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.appendPlainText(f"[{now}] {line}")

    def closeEvent(self, event):
        self.worker.stop()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = FactoryTestWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
