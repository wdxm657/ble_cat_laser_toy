# -*- coding: utf-8 -*-
"""BLE scanner, connection, and command worker."""

import asyncio
import threading
from typing import Dict

from PyQt5 import QtCore

from .constants import (
    CTRL_CMD_DEVICE_REBOOT,
    CTRL_CMD_FACTORY_TEST_ENTER,
    CTRL_FACTORY_TEST_MODULE_LASER,
    CTRL_FACTORY_TEST_MODULE_MOTOR,
    CTRL_FACTORY_TEST_MODULE_RADAR,
    CTRL_LOG_RAW_BYTES,
    CTRL_RX_RAW_BYTES,
    CTRL_TX_RAW_BYTES,
)
from .deps import BLEAK_IMPORT_ERROR, BleakClient, BleakScanner
from .models import ScanDevice
from .protocol import build_ctrl_cmd, decode_ctrl_frame, find_characteristic, format_manufacturer_data

class BleWorker(QtCore.QObject):
    devices_changed = QtCore.pyqtSignal(list)
    log_line = QtCore.pyqtSignal(str)
    firmware_log_line = QtCore.pyqtSignal(str)
    status_changed = QtCore.pyqtSignal(str, bool)
    ctrl_frame = QtCore.pyqtSignal(dict)

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

    def scan(self, timeout_s: float = 1.0):
        self._schedule(self._scan(timeout_s))

    def connect(self, address: str):
        self._schedule(self._connect(address))

    def disconnect(self):
        self._schedule(self._disconnect())

    def send_factory_test_enter(self):
        self._schedule(self._send_cmd(CTRL_CMD_FACTORY_TEST_ENTER, '进入测试模式'))

    def send_factory_test_module(self, module: int, enable: bool):
        labels = {
            CTRL_FACTORY_TEST_MODULE_RADAR: '雷达',
            CTRL_FACTORY_TEST_MODULE_MOTOR: '电机',
            CTRL_FACTORY_TEST_MODULE_LASER: '激光灯',
        }
        label = labels.get(module, f'模块0x{module:02X}')
        suffix = '开启' if enable else '关闭'
        payload = bytes([module & 0xFF, 1 if enable else 0])
        self._schedule(self._send_cmd(CTRL_CMD_FACTORY_TEST_ENTER, f'{label}{suffix}', payload))

    def send_reboot(self):
        self._schedule(self._send_cmd(CTRL_CMD_DEVICE_REBOOT, '重启设备'))

    async def _scan(self, timeout_s: float):
        if BLEAK_IMPORT_ERROR is not None:
            self.log_line.emit(f'Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}')
            return

        self.log_line.emit(f'开始扫描 W2MLaserTOY 设备，{timeout_s:.0f}s...')
        found: Dict[str, ScanDevice] = {}

        def on_adv(device, adv):
            name = adv.local_name or getattr(device, 'name', None) or ''
            if not name.startswith('W2MLaserTOY'):
                return
            rssi = getattr(adv, 'rssi', None)
            if rssi is None:
                rssi = getattr(device, 'rssi', None)
            found[device.address] = ScanDevice(
                name=name,
                address=device.address,
                rssi=rssi,
                manufacturer_data=format_manufacturer_data(getattr(adv, 'manufacturer_data', {}) or {}),
            )
            self._devices = dict(found)
            self.devices_changed.emit(list(found.values()))

        scanner = BleakScanner(detection_callback=on_adv)
        try:
            await scanner.start()
            await asyncio.sleep(timeout_s)
            await scanner.stop()
        except Exception as ex:
            self.log_line.emit(f'扫描失败: {ex}')
            return

        self._devices = dict(found)
        self.devices_changed.emit(list(found.values()))
        self.log_line.emit(f'扫描完成，找到 {len(found)} 个 W2MLaserTOY 设备')

    async def _connect(self, address: str):
        if BLEAK_IMPORT_ERROR is not None:
            self.log_line.emit(f'Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}')
            return
        if not address:
            self.log_line.emit('请先选择设备')
            return
        if self._client and self._client.is_connected:
            self.log_line.emit('已连接，请先断开')
            return

        self.log_line.emit(f'连接中: {address}')

        def on_disconnect(_client):
            self._client = None
            self._rx_char = None
            self.status_changed.emit('已断开', False)
            self.log_line.emit('设备已断开')

        client = BleakClient(address, disconnected_callback=on_disconnect)
        try:
            await client.connect()
            get_services = getattr(client, 'get_services', None)
            if get_services:
                out = get_services()
                if asyncio.iscoroutine(out):
                    await out
            else:
                _ = client.services

            self._rx_char = find_characteristic(client, CTRL_RX_RAW_BYTES)
            tx_char = find_characteristic(client, CTRL_TX_RAW_BYTES)
            log_char = find_characteristic(client, CTRL_LOG_RAW_BYTES)
            if self._rx_char is None:
                raise RuntimeError('未找到 Ctrl RX 写特征')
            if tx_char is not None:
                await client.start_notify(tx_char, self._on_ctrl_notify)
            if log_char is not None:
                await client.start_notify(log_char, self._on_log_notify)
            else:
                self.log_line.emit('未找到 BLE_LOG_D 日志特征，仅显示控制响应')

            self._client = client
            self.status_changed.emit(f'已连接 {address}', True)
            self.log_line.emit('连接成功')
        except Exception as ex:
            try:
                await client.disconnect()
            except Exception:
                pass
            self._client = None
            self._rx_char = None
            self.status_changed.emit('连接失败', False)
            self.log_line.emit(f'连接失败: {ex}')

    async def _disconnect(self):
        client = self._client
        self._client = None
        self._rx_char = None
        if client is None:
            self.status_changed.emit('未连接', False)
            return
        try:
            if client.is_connected:
                await client.disconnect()
        except Exception as ex:
            self.log_line.emit(f'断开失败: {ex}')
            return
        self.status_changed.emit('已断开', False)
        self.log_line.emit('已手动断开')

    async def _send_cmd(self, cmd_id: int, label: str, payload: bytes = b''):
        client = self._client
        if client is None or not client.is_connected or self._rx_char is None:
            self.log_line.emit('未连接，无法发送命令')
            return
        seq = self._seq & 0xFF
        self._seq = (self._seq + 1) & 0xFF
        frame = build_ctrl_cmd(cmd_id, seq, payload)
        try:
            await client.write_gatt_char(self._rx_char, frame, response=False)
            self.log_line.emit(f'已发送 {label}: cmd=0x{cmd_id:02X} seq={seq}')
        except Exception as ex:
            self.log_line.emit(f'发送失败 {label}: {ex}')

    def _on_ctrl_notify(self, _sender, data: bytearray):
        frame = decode_ctrl_frame(bytes(data))
        self.ctrl_frame.emit(frame)
        self.log_line.emit(frame['text'])

    def _on_log_notify(self, _sender, data: bytearray):
        self._log_buf.extend(bytes(data))
        while b'\n' in self._log_buf:
            idx = self._log_buf.index(0x0A)
            raw = bytes(self._log_buf[:idx]).rstrip(b'\r')
            del self._log_buf[: idx + 1]
            if raw:
                self.firmware_log_line.emit(raw.decode('utf-8', errors='replace'))
        if len(self._log_buf) > 512:
            raw = bytes(self._log_buf)
            self._log_buf.clear()
            self.firmware_log_line.emit(raw.decode('utf-8', errors='replace'))



