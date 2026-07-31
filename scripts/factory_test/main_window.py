# -*- coding: utf-8 -*-
"""Main PyQt window for W2MLaserTOY factory test."""

import csv
import datetime as _dt
import os
from typing import Dict, Optional

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import Qt

from .ble_worker import BleWorker
from .camera_dialog import CameraQrScanDialog
from .constants import (
    CTRL_CMD_FACTORY_TEST_ENTER,
    CTRL_FACTORY_TEST_MODULE_LASER,
    CTRL_FACTORY_TEST_MODULE_MOTOR,
    CTRL_FACTORY_TEST_MODULE_RADAR,
    CTRL_MSG_TYPE_RSP,
    DEFAULT_RESULT_CSV,
    DEFAULT_RESULT_XLSX,
    RESULT_HEADERS,
)
from .deps import BLEAK_IMPORT_ERROR, OPENPYXL_IMPORT_ERROR, Font, PatternFill, Workbook
from .models import ScanDevice

class FactoryTestWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = BleWorker()
        self.devices: Dict[str, ScanDevice] = {}
        self.connected_address: Optional[str] = None
        self.factory_ready = False
        self.tested_mfr_set = set()
        self.module_results = {
            'radar': None,
            'motor': None,
            'laser': None,
        }
        self.ui_font_size = 20
        self.result_csv_path = DEFAULT_RESULT_CSV
        self.result_xlsx_path = DEFAULT_RESULT_XLSX
        self.qr_dialog = None

        self.setWindowTitle('W2MLaserTOY 组装工厂测试工具')
        self.resize(1280, 860)
        self._build_ui()
        self._connect_signals()
        self._set_connected(False)
        self._update_result_path_label()
        self._refresh_csv_preview()

        if BLEAK_IMPORT_ERROR is not None:
            self._append_log(f'Bleak 未安装或导入失败: {BLEAK_IMPORT_ERROR}')

    def _build_stylesheet(self, font_px: int) -> str:
        return f"""
            QWidget {{
                font-size: {font_px}px;
                color: #e5e7eb;
                background: #0f172a;
            }}
            QGroupBox {{
                font-weight: 600;
                border: 1px solid #334155;
                border-radius: 6px;
                margin-top: 10px;
                background: #111827;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #cbd5e1;
            }}
            QPushButton {{
                min-width: 76px;
                padding: 6px 10px;
                border: 1px solid #475569;
                border-radius: 5px;
                background: #1f2937;
                color: #e5e7eb;
            }}
            QPushButton:hover {{ background: #334155; }}
            QPushButton:pressed {{ background: #475569; }}
            QPushButton:disabled {{
                color: #64748b;
                background: #1e293b;
                border-color: #334155;
            }}
            QPlainTextEdit, QTableWidget, QTreeWidget, QComboBox, QSpinBox {{
                border: 1px solid #334155;
                border-radius: 4px;
                background: #0b1220;
                color: #e5e7eb;
            }}
            QPlainTextEdit, QTableWidget, QTreeWidget {{
                selection-background-color: #2563eb;
                selection-color: #ffffff;
            }}
            QTreeWidget {{
                alternate-background-color: #111827;
            }}
            QTreeWidget::item {{
                padding: 2px 4px;
                color: #e5e7eb;
            }}
            QTreeWidget::item:alternate {{
                background: #111827;
            }}
            QTreeWidget::item:selected {{
                background: #1d4ed8;
                color: #ffffff;
            }}
            QTreeWidget:disabled {{
                background: #0b1220;
                color: #cbd5e1;
            }}
            QTreeWidget::item:disabled {{
                color: #cbd5e1;
            }}
            QHeaderView::section {{
                background: #1e293b;
                color: #e2e8f0;
                border: 1px solid #334155;
                padding: 6px 8px;
                font-weight: 600;
            }}
            QTableWidget::item:selected {{
                background: #1d4ed8;
                color: #ffffff;
            }}
            QComboBox::drop-down, QSpinBox::up-button, QSpinBox::down-button {{
                border-left: 1px solid #334155;
                width: 20px;
            }}
            QComboBox QAbstractItemView {{
                background: #0b1220;
                color: #e5e7eb;
                selection-background-color: #2563eb;
                border: 1px solid #334155;
            }}
            QLabel#statusLabel {{
                padding: 6px 10px;
                border-radius: 5px;
                background: #1e293b;
                color: #bfdbfe;
                font-weight: 600;
            }}
            QSplitter::handle {{
                background: #334155;
            }}
        """

    def _apply_ui_style(self):
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app_font = app.font()
            app_font.setPointSize(self.ui_font_size)
            app.setFont(app_font)
        self.setStyleSheet(self._build_stylesheet(self.ui_font_size))

    def _on_font_size_changed(self, value: int):
        self.ui_font_size = value
        self._apply_ui_style()
        self.device_tree.resizeColumnToContents(0)
        self.device_tree.resizeColumnToContents(1)
        self.device_tree.resizeColumnToContents(2)
        self.device_tree.resizeColumnToContents(3)
        self.csv_preview_table.resizeRowsToContents()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)
        self._apply_ui_style()
        self.setStyleSheet("""
            QWidget {
                font-size: 13px;
                color: #e5e7eb;
                background: #0f172a;
            }
            QGroupBox {
                font-weight: 600;
                border: 1px solid #334155;
                border-radius: 6px;
                margin-top: 10px;
                background: #111827;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #cbd5e1;
            }
            QPushButton {
                min-width: 76px;
                padding: 6px 10px;
                border: 1px solid #475569;
                border-radius: 5px;
                background: #1f2937;
                color: #e5e7eb;
            }
            QPushButton:hover { background: #334155; }
            QPushButton:pressed { background: #475569; }
            QPushButton:disabled {
                color: #64748b;
                background: #1e293b;
                border-color: #334155;
            }
            QPlainTextEdit, QTableWidget, QTreeWidget, QComboBox {
                border: 1px solid #334155;
                border-radius: 4px;
                background: #0b1220;
                color: #e5e7eb;
            }
            QPlainTextEdit, QTableWidget, QTreeWidget {
                selection-background-color: #2563eb;
                selection-color: #ffffff;
            }
            QHeaderView::section {
                background: #1e293b;
                color: #e2e8f0;
                border: 1px solid #334155;
                padding: 6px 8px;
                font-weight: 600;
            }
            QTableWidget::item:selected, QTreeWidget::item:selected {
                background: #1d4ed8;
                color: #ffffff;
            }
            QComboBox::drop-down {
                border-left: 1px solid #334155;
                width: 20px;
            }
            QComboBox QAbstractItemView {
                background: #0b1220;
                color: #e5e7eb;
                selection-background-color: #2563eb;
                border: 1px solid #334155;
            }
            QLabel#statusLabel {
                padding: 6px 10px;
                border-radius: 5px;
                background: #1e293b;
                color: #bfdbfe;
                font-weight: 600;
            }
            QSplitter::handle {
                background: #334155;
            }
        """)

        device_box = QtWidgets.QGroupBox('设备选择')
        self._apply_ui_style()
        device_layout = QtWidgets.QVBoxLayout(device_box)
        scan_row = QtWidgets.QHBoxLayout()
        self.scan_btn = QtWidgets.QPushButton('扫描 W2MLaserTOY 设备')
        self.scan_qr_btn = QtWidgets.QPushButton('摄像头扫码')
        self.device_combo = QtWidgets.QComboBox()
        self.device_combo.setMinimumWidth(380)
        self.connect_btn = QtWidgets.QPushButton('连接')
        self.disconnect_btn = QtWidgets.QPushButton('断开')
        scan_row.addWidget(self.scan_btn)
        scan_row.addWidget(self.scan_qr_btn)
        scan_row.addWidget(self.device_combo, 1)
        scan_row.addWidget(self.connect_btn)
        scan_row.addWidget(self.disconnect_btn)
        device_layout.addLayout(scan_row)

        self.status_label = QtWidgets.QLabel('未连接')
        self.status_label.setObjectName('statusLabel')
        device_layout.addWidget(self.status_label)
        font_row = QtWidgets.QHBoxLayout()
        font_row.setSpacing(8)
        self.font_size_label = QtWidgets.QLabel('字体大小')
        self.font_size_spin = QtWidgets.QSpinBox()
        self.font_size_spin.setRange(15, 25)
        self.font_size_spin.setValue(self.ui_font_size)
        self.font_size_spin.setSuffix(' px')
        self.font_size_spin.setFixedWidth(96)
        font_row.addWidget(self.font_size_label)
        font_row.addWidget(self.font_size_spin)
        font_row.addStretch(1)
        device_layout.addLayout(font_row)

        self.device_tree = QtWidgets.QTreeWidget()
        self.device_tree.setHeaderLabels(['名称', 'MAC/地址', 'RSSI', '是否测试过', 'MANUFACTURER_DATA'])
        self.device_tree.setRootIsDecorated(False)
        self.device_tree.setAlternatingRowColors(True)
        self.device_tree.setUniformRowHeights(True)
        self.device_tree.headerItem().setTextAlignment(2, Qt.AlignCenter)
        self.device_tree.headerItem().setTextAlignment(3, Qt.AlignCenter)
        self.device_tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.device_tree.header().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        device_layout.addWidget(self.device_tree, 2)
        root.addWidget(device_box, 3)

        action_box = QtWidgets.QGroupBox('设备操作')
        action_box_layout = QtWidgets.QVBoxLayout(action_box)
        action_row = QtWidgets.QHBoxLayout()
        self.start_test_btn = QtWidgets.QPushButton('开始测试')
        self.reboot_btn = QtWidgets.QPushButton('重启设备')
        self.clear_log_btn = QtWidgets.QPushButton('清空日志')
        self.criteria_help_btn = QtWidgets.QPushButton('测试结果判据说明')
        self.usage_help_btn = QtWidgets.QPushButton('使用说明')
        self.start_test_btn.setMinimumHeight(40)
        self.reboot_btn.setMinimumHeight(40)
        action_row.addWidget(self.start_test_btn)
        action_row.addWidget(self.reboot_btn)
        action_row.addStretch(1)
        action_row.addWidget(self.clear_log_btn)
        action_row.addWidget(self.criteria_help_btn)
        action_row.addWidget(self.usage_help_btn)
        action_box_layout.addLayout(action_row)
        root.addWidget(action_box)

        module_box = QtWidgets.QGroupBox('模块控制')
        test_ops_row = QtWidgets.QHBoxLayout()
        test_ops_row.setSpacing(10)

        module_row = QtWidgets.QVBoxLayout(module_box)
        module_row.setSpacing(10)
        self.radar_on_btn = QtWidgets.QPushButton('雷达开启')
        self.radar_off_btn = QtWidgets.QPushButton('雷达关闭')
        self.motor_on_btn = QtWidgets.QPushButton('电机开启')
        self.motor_off_btn = QtWidgets.QPushButton('电机关闭')
        self.laser_on_btn = QtWidgets.QPushButton('激光灯开启')
        self.laser_off_btn = QtWidgets.QPushButton('激光灯关闭')
        for btn in (self.radar_on_btn, self.radar_off_btn, self.motor_on_btn, self.motor_off_btn, self.laser_on_btn, self.laser_off_btn):
            btn.setMinimumHeight(34)
        self._add_button_pair(module_row, '雷达控制', self.radar_on_btn, self.radar_off_btn)
        self._add_button_pair(module_row, '电机控制', self.motor_on_btn, self.motor_off_btn)
        self._add_button_pair(module_row, '激光灯控制', self.laser_on_btn, self.laser_off_btn)
        test_ops_row.addWidget(module_box, 1)

        result_box = QtWidgets.QGroupBox('测试结果确认')
        result_row = QtWidgets.QVBoxLayout(result_box)
        result_row.setSpacing(10)
        self.radar_ok_btn = QtWidgets.QPushButton('雷达成功')
        self.radar_fail_btn = QtWidgets.QPushButton('雷达失败')
        self.motor_ok_btn = QtWidgets.QPushButton('电机成功')
        self.motor_fail_btn = QtWidgets.QPushButton('电机失败')
        self.laser_ok_btn = QtWidgets.QPushButton('激光灯成功')
        self.laser_fail_btn = QtWidgets.QPushButton('激光灯失败')
        self.confirm_btn = QtWidgets.QPushButton('测试结果保存')
        self.export_excel_btn = QtWidgets.QPushButton('CSV 转 Excel')
        for btn in (self.radar_ok_btn, self.radar_fail_btn, self.motor_ok_btn, self.motor_fail_btn, self.laser_ok_btn, self.laser_fail_btn, self.confirm_btn, self.export_excel_btn):
            btn.setMinimumHeight(34)
        self._add_button_pair(result_row, '雷达结果', self.radar_ok_btn, self.radar_fail_btn)
        self._add_button_pair(result_row, '电机结果', self.motor_ok_btn, self.motor_fail_btn)
        self._add_button_pair(result_row, '激光灯结果', self.laser_ok_btn, self.laser_fail_btn)
        result_action_row = QtWidgets.QHBoxLayout()
        result_action_row.setSpacing(10)
        result_action_row.addWidget(self.confirm_btn)
        result_action_row.addWidget(self.export_excel_btn)
        result_row.addLayout(result_action_row)
        self.result_path_label = QtWidgets.QLabel()
        self.result_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.choose_csv_btn = QtWidgets.QPushButton('选择 CSV')
        self.choose_excel_btn = QtWidgets.QPushButton('选择 Excel')
        for btn in (self.choose_csv_btn, self.choose_excel_btn):
            btn.setMinimumHeight(30)
        path_row = QtWidgets.QHBoxLayout()
        path_row.setSpacing(8)
        path_row.addWidget(self.result_path_label, 1)
        path_row.addWidget(self.choose_csv_btn)
        path_row.addWidget(self.choose_excel_btn)
        result_panel = QtWidgets.QWidget()
        result_layout = QtWidgets.QVBoxLayout(result_panel)
        result_layout.setContentsMargins(0, 0, 0, 0)
        result_layout.addWidget(result_box)
        result_layout.addLayout(path_row)
        test_ops_row.addWidget(result_panel, 1)
        root.addLayout(test_ops_row)

        preview_box = QtWidgets.QGroupBox('CSV 实时预览')
        preview_layout = QtWidgets.QVBoxLayout(preview_box)
        self.csv_preview_table = QtWidgets.QTableWidget()
        self.csv_preview_table.setColumnCount(len(RESULT_HEADERS))
        self.csv_preview_table.setHorizontalHeaderLabels(RESULT_HEADERS)
        self.csv_preview_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.csv_preview_table.setAlternatingRowColors(True)
        self.csv_preview_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.csv_preview_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        preview_layout.addWidget(self.csv_preview_table)
        root.addWidget(preview_box, 2)

        log_splitter = QtWidgets.QSplitter(Qt.Horizontal)
        host_log_box = QtWidgets.QGroupBox('上位机日志')
        host_log_layout = QtWidgets.QVBoxLayout(host_log_box)
        self.host_log_text = QtWidgets.QPlainTextEdit()
        self.host_log_text.setReadOnly(True)
        self.host_log_text.setPlaceholderText('扫描、连接、按钮操作和控制响应会显示在这里')
        host_log_layout.addWidget(self.host_log_text)

        firmware_log_box = QtWidgets.QGroupBox('固件上传日志')
        firmware_log_layout = QtWidgets.QVBoxLayout(firmware_log_box)
        self.firmware_log_text = QtWidgets.QPlainTextEdit()
        self.firmware_log_text.setReadOnly(True)
        self.firmware_log_text.setPlaceholderText('BLE_LOG_D 固件日志会显示在这里')
        firmware_log_layout.addWidget(self.firmware_log_text)

        log_splitter.addWidget(host_log_box)
        log_splitter.addWidget(firmware_log_box)
        log_splitter.setStretchFactor(0, 1)
        log_splitter.setStretchFactor(1, 1)
        root.addWidget(log_splitter, 3)

    def _add_button_pair(self, parent_layout, title: str, left_btn, right_btn):
        box = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QHBoxLayout(box)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(left_btn)
        layout.addWidget(right_btn)
        parent_layout.addWidget(box, 1)

    def _connect_signals(self):
        self.scan_btn.clicked.connect(lambda: self.worker.scan())
        self.scan_qr_btn.clicked.connect(self._on_scan_qr_clicked)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.disconnect_btn.clicked.connect(self.worker.disconnect)
        self.start_test_btn.clicked.connect(self._on_start_test_clicked)
        self.reboot_btn.clicked.connect(self.worker.send_reboot)
        self.clear_log_btn.clicked.connect(self._clear_logs)
        self.criteria_help_btn.clicked.connect(self._show_criteria_help)
        self.usage_help_btn.clicked.connect(self._show_usage_help)
        self.device_tree.itemDoubleClicked.connect(self._on_device_double_clicked)
        self.font_size_spin.valueChanged.connect(self._on_font_size_changed)

        self.radar_on_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_RADAR, True))
        self.radar_off_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_RADAR, False))
        self.motor_on_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_MOTOR, True))
        self.motor_off_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_MOTOR, False))
        self.laser_on_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_LASER, True))
        self.laser_off_btn.clicked.connect(lambda: self._send_module_ctrl(CTRL_FACTORY_TEST_MODULE_LASER, False))

        self.radar_ok_btn.clicked.connect(lambda: self._set_module_result('radar', '成功'))
        self.radar_fail_btn.clicked.connect(lambda: self._set_module_result('radar', '失败'))
        self.motor_ok_btn.clicked.connect(lambda: self._set_module_result('motor', '成功'))
        self.motor_fail_btn.clicked.connect(lambda: self._set_module_result('motor', '失败'))
        self.laser_ok_btn.clicked.connect(lambda: self._set_module_result('laser', '成功'))
        self.laser_fail_btn.clicked.connect(lambda: self._set_module_result('laser', '失败'))
        self.confirm_btn.clicked.connect(self._on_confirm_clicked)
        self.export_excel_btn.clicked.connect(self._on_export_excel_clicked)
        self.choose_csv_btn.clicked.connect(self._on_choose_csv_clicked)
        self.choose_excel_btn.clicked.connect(self._on_choose_excel_clicked)

        self.worker.devices_changed.connect(self._on_devices_changed)
        self.worker.log_line.connect(self._append_log)
        self.worker.firmware_log_line.connect(self._append_firmware_log)
        self.worker.status_changed.connect(self._on_status_changed)
        self.worker.ctrl_frame.connect(self._on_ctrl_frame)

    def _on_connect_clicked(self):
        address = self.device_combo.currentData()
        self.worker.connect(address or '')

    def _on_device_double_clicked(self, item, _column):
        address = item.data(0, Qt.UserRole)
        idx = self.device_combo.findData(address)
        if idx >= 0:
            self.device_combo.setCurrentIndex(idx)

    def _on_scan_qr_clicked(self):
        if self.qr_dialog is not None:
            self.qr_dialog.close()
        self.qr_dialog = CameraQrScanDialog(self)
        self.qr_dialog.device_matched.connect(self._on_qr_device_matched)
        self.qr_dialog.finished.connect(lambda _result: setattr(self, 'qr_dialog', None))
        self.qr_dialog.update_devices(list(self.devices.values()))
        self.qr_dialog.show()
        self.worker.scan(3.0)
        self._append_log('已打开摄像头扫码窗口，并开始扫描蓝牙设备')

    def _on_qr_device_matched(self, address: str, qr_value: str):
        idx = self.device_combo.findData(address)
        if idx >= 0:
            self.device_combo.setCurrentIndex(idx)
        for row_idx in range(self.device_tree.topLevelItemCount()):
            item = self.device_tree.topLevelItem(row_idx)
            if item.data(0, Qt.UserRole) == address:
                self.device_tree.setCurrentItem(item)
                self.device_tree.scrollToItem(item)
                break
        self._append_log(f'二维码匹配到设备: {address}, MANUFACTURER_DATA={qr_value}')

    def _on_start_test_clicked(self):
        self.factory_ready = False
        self._reset_module_results()
        self.worker.send_factory_test_enter()
        self._append_log('已发送进入测试模式命令，等待设备响应')
        self._update_buttons()

    def _send_module_ctrl(self, module: int, enable: bool):
        if not self.factory_ready:
            self._append_log('设备尚未进入测试状态，不能操作模块')
            return
        self.worker.send_factory_test_module(module, enable)

    def _module_name(self, module: str) -> str:
        return {'radar': '雷达', 'motor': '电机', 'laser': '激光灯'}[module]

    def _on_ctrl_frame(self, frame: dict):
        if frame.get('short'):
            return
        if frame.get('msg_type') != CTRL_MSG_TYPE_RSP or frame.get('cmd_id') != CTRL_CMD_FACTORY_TEST_ENTER:
            return
        payload = frame.get('payload', b'') or b''
        status = frame.get('status')
        if len(payload) <= 1:
            if status == 'OK':
                self.factory_ready = True
                self._append_log('设备已进入测试状态')
            else:
                self.factory_ready = False
                self._append_log(f'进入测试状态失败: {status}')
        elif len(payload) >= 3 and status == 'OK':
            module = payload[1]
            enable = bool(payload[2])
            module_name = {
                CTRL_FACTORY_TEST_MODULE_RADAR: '雷达',
                CTRL_FACTORY_TEST_MODULE_MOTOR: '电机',
                CTRL_FACTORY_TEST_MODULE_LASER: '激光灯',
            }.get(module, f'模块0x{module:02X}')
            self._append_log(f'{module_name}{"开启" if enable else "关闭"}已执行')
        self._update_buttons()

    def _on_devices_changed(self, devices):
        current = self.device_combo.currentData()
        self.devices = {d.address: d for d in devices}
        self._refresh_tested_mfr_set()

        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        for dev in devices:
            self.device_combo.addItem(f'{dev.name}  {dev.address}', dev.address)
        if current:
            idx = self.device_combo.findData(current)
            if idx >= 0:
                self.device_combo.setCurrentIndex(idx)
        self.device_combo.blockSignals(False)

        self.device_tree.clear()
        mono_font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        for row_idx, dev in enumerate(devices):
            tested = '已测' if dev.manufacturer_data and dev.manufacturer_data in self.tested_mfr_set else '未测'
            item = QtWidgets.QTreeWidgetItem([
                dev.name,
                dev.address,
                '' if dev.rssi is None else str(dev.rssi),
                tested,
                dev.manufacturer_data,
            ])
            row_bg = QtGui.QBrush(QtGui.QColor('#0b1220' if row_idx % 2 == 0 else '#111827'))
            row_fg = QtGui.QBrush(QtGui.QColor('#e5e7eb'))
            for col_idx in range(5):
                item.setBackground(col_idx, row_bg)
                item.setForeground(col_idx, row_fg)
            if tested == '已测':
                item.setForeground(3, QtGui.QBrush(QtGui.QColor('#86efac')))
            else:
                item.setForeground(3, QtGui.QBrush(QtGui.QColor('#fbbf24')))
            item.setFont(1, mono_font)
            item.setFont(4, mono_font)
            item.setTextAlignment(0, Qt.AlignVCenter | Qt.AlignLeft)
            item.setTextAlignment(1, Qt.AlignVCenter | Qt.AlignLeft)
            item.setTextAlignment(2, Qt.AlignCenter)
            item.setTextAlignment(3, Qt.AlignCenter)
            item.setTextAlignment(4, Qt.AlignVCenter | Qt.AlignLeft)
            item.setData(0, Qt.UserRole, dev.address)
            self.device_tree.addTopLevelItem(item)
        if self.device_combo.count() > 0 and self.device_combo.currentIndex() < 0:
            self.device_combo.setCurrentIndex(0)
        self.device_tree.resizeColumnToContents(0)
        self.device_tree.resizeColumnToContents(1)
        self.device_tree.resizeColumnToContents(2)
        self.device_tree.resizeColumnToContents(3)
        if self.qr_dialog is not None:
            self.qr_dialog.update_devices(devices)

    def _on_status_changed(self, text: str, connected: bool):
        self.status_label.setText(text)
        if connected:
            self.connected_address = self.device_combo.currentData()
        else:
            self.connected_address = None
            self.factory_ready = False
            self._reset_module_results()
        self._set_connected(connected)

    def _set_connected(self, connected: bool):
        self.scan_btn.setEnabled(not connected)
        self.scan_qr_btn.setEnabled(not connected)
        self.device_combo.setEnabled(not connected)
        self.device_tree.setEnabled(not connected)
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.start_test_btn.setEnabled(connected)
        self.reboot_btn.setEnabled(connected)
        self._update_buttons()

    def _reset_module_results(self):
        self.module_results = {'radar': None, 'motor': None, 'laser': None}

    def _set_module_result(self, module: str, result: str):
        if not self.factory_ready:
            self._append_log('设备尚未进入测试状态，不能记录结果')
            return
        self.module_results[module] = result
        self._append_log(f'{self._module_name(module)}测试结果：{result}')
        self._update_buttons()

    def _update_buttons(self):
        can_control = self.connected_address is not None and self.factory_ready
        for btn in (
            self.radar_on_btn, self.radar_off_btn,
            self.motor_on_btn, self.motor_off_btn,
            self.laser_on_btn, self.laser_off_btn,
            self.radar_ok_btn, self.radar_fail_btn,
            self.motor_ok_btn, self.motor_fail_btn,
            self.laser_ok_btn, self.laser_fail_btn,
        ):
            btn.setEnabled(can_control)
        self.confirm_btn.setEnabled(
            can_control and all(self.module_results[k] in ('成功', '失败') for k in ('radar', 'motor', 'laser'))
        )

    def _current_device(self) -> Optional[ScanDevice]:
        if not self.connected_address:
            return None
        return self.devices.get(self.connected_address) or ScanDevice(name='', address=self.connected_address, rssi=None, manufacturer_data='')

    def _build_result_row(self, dev: ScanDevice):
        return [
            _dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            dev.name,
            dev.address,
            dev.manufacturer_data,
            self.module_results['radar'] or '',
            self.module_results['motor'] or '',
            self.module_results['laser'] or '',
            'PASS' if all(self.module_results[k] == '成功' for k in ('radar', 'motor', 'laser')) else 'FAIL',
        ]

    def _short_path(self, path: str) -> str:
        base = os.path.basename(path)
        folder = os.path.basename(os.path.dirname(path))
        if folder:
            return os.path.join(folder, base)
        return base or path

    def _update_result_path_label(self):
        csv_short = self._short_path(self.result_csv_path)
        excel_short = self._short_path(self.result_xlsx_path)
        self.result_path_label.setText(f'CSV: {csv_short}    Excel: {excel_short}')
        self.result_path_label.setToolTip(f'CSV: {self.result_csv_path}\nExcel: {self.result_xlsx_path}')

    def _normalize_save_path(self, path: str, suffix: str) -> str:
        if not path:
            return path
        root, ext = os.path.splitext(path)
        if not ext:
            return root + suffix
        return path

    def _on_choose_csv_clicked(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            '选择 CSV 保存路径',
            self.result_csv_path,
            'CSV Files (*.csv);;All Files (*)',
        )
        path = self._normalize_save_path(path, '.csv')
        if not path:
            return
        self.result_csv_path = path
        self._update_result_path_label()
        self._refresh_csv_preview()
        self._on_devices_changed(list(self.devices.values()))

    def _on_choose_excel_clicked(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            '选择 Excel 保存路径',
            self.result_xlsx_path,
            'Excel Files (*.xlsx);;All Files (*)',
        )
        path = self._normalize_save_path(path, '.xlsx')
        if not path:
            return
        self.result_xlsx_path = path
        self._update_result_path_label()

    def _read_csv_rows(self):
        if not os.path.exists(self.result_csv_path):
            return []
        with open(self.result_csv_path, 'r', newline='', encoding='utf-8-sig') as fp:
            rows = list(csv.reader(fp))
        if not rows:
            return []
        if rows[0] == RESULT_HEADERS:
            return rows[1:]
        return rows

    def _write_csv_rows(self, rows):
        os.makedirs(os.path.dirname(os.path.abspath(self.result_csv_path)), exist_ok=True)
        with open(self.result_csv_path, 'w', newline='', encoding='utf-8-sig') as fp:
            writer = csv.writer(fp)
            writer.writerow(RESULT_HEADERS)
            writer.writerows(rows)

    def _refresh_tested_mfr_set(self, rows=None):
        if rows is None:
            rows = self._read_csv_rows()
        mfr_col = RESULT_HEADERS.index('MANUFACTURER_DATA')
        self.tested_mfr_set = {
            row[mfr_col]
            for row in rows
            if len(row) > mfr_col and row[mfr_col]
        }

    def _save_csv_record(self, dev: ScanDevice):
        rows = self._read_csv_rows()
        if dev.manufacturer_data:
            mfr_col = RESULT_HEADERS.index('MANUFACTURER_DATA')
            rows = [row for row in rows if len(row) <= mfr_col or row[mfr_col] != dev.manufacturer_data]
        rows.append(self._build_result_row(dev))
        self._write_csv_rows(rows)
        self._refresh_csv_preview(rows)
        self._refresh_tested_mfr_set(rows)
        self._on_devices_changed(list(self.devices.values()))

    def _refresh_csv_preview(self, rows=None):
        if rows is None:
            rows = self._read_csv_rows()
        self._refresh_tested_mfr_set(rows)
        self.csv_preview_table.setRowCount(len(rows))
        for row_idx, row in enumerate(rows):
            normalized = list(row[:len(RESULT_HEADERS)]) + [''] * max(0, len(RESULT_HEADERS) - len(row))
            for col_idx, value in enumerate(normalized[:len(RESULT_HEADERS)]):
                item = QtWidgets.QTableWidgetItem(str(value))
                self.csv_preview_table.setItem(row_idx, col_idx, item)
        self.csv_preview_table.resizeRowsToContents()

    def _dedupe_rows_by_manufacturer_data(self, rows):
        mfr_col = RESULT_HEADERS.index('MANUFACTURER_DATA')
        out = []
        index_by_mfr = {}
        for row in rows:
            normalized = list(row[:len(RESULT_HEADERS)]) + [''] * max(0, len(RESULT_HEADERS) - len(row))
            mfr = normalized[mfr_col]
            if mfr:
                old_idx = index_by_mfr.get(mfr)
                if old_idx is not None:
                    out.pop(old_idx)
                    index_by_mfr = {
                        existing[mfr_col]: idx
                        for idx, existing in enumerate(out)
                        if len(existing) > mfr_col and existing[mfr_col]
                    }
                index_by_mfr[mfr] = len(out)
            out.append(normalized[:len(RESULT_HEADERS)])
        return out

    def _export_csv_to_excel(self):
        if OPENPYXL_IMPORT_ERROR is not None:
            raise RuntimeError(f'openpyxl 未安装或导入失败: {OPENPYXL_IMPORT_ERROR}')
        rows = self._dedupe_rows_by_manufacturer_data(self._read_csv_rows())
        self._write_csv_rows(rows)
        self._refresh_csv_preview(rows)

        wb = Workbook()
        ws = wb.active
        ws.title = 'Factory Test'
        ws.append(RESULT_HEADERS)
        for row in rows:
            ws.append(row)

        header_fill = PatternFill('solid', fgColor='D9EAF7')
        for cell in ws[1]:
            cell.font = Font(bold=True)
            cell.fill = header_fill
        ws.freeze_panes = 'A2'
        ws.auto_filter.ref = ws.dimensions
        for col_cells in ws.columns:
            column_letter = col_cells[0].column_letter
            max_len = max(len('' if cell.value is None else str(cell.value)) for cell in col_cells)
            ws.column_dimensions[column_letter].width = min(max(max_len + 2, 10), 48)

        os.makedirs(os.path.dirname(os.path.abspath(self.result_xlsx_path)), exist_ok=True)
        wb.save(self.result_xlsx_path)

    def _on_confirm_clicked(self):
        dev = self._current_device()
        if dev is None:
            self._append_log('未连接设备，无法保存测试结果')
            return
        if not self.factory_ready:
            self._append_log('请先进入测试状态')
            return
        if not all(self.module_results[k] in ('成功', '失败') for k in ('radar', 'motor', 'laser')):
            self._append_log('三个模块都需要先选择成功/失败')
            return
        try:
            self._save_csv_record(dev)
        except Exception as ex:
            self._append_log(f'保存 CSV 失败: {ex}')
            return
        self._append_log(f'测试结果已保存到 CSV: {self.result_csv_path}')
        self._reset_module_results()
        self._update_buttons()

    def _on_export_excel_clicked(self):
        try:
            self._export_csv_to_excel()
        except Exception as ex:
            self._append_log(f'CSV 转 Excel 失败: {ex}')
            return
        self._append_log(f'CSV 已转换为 Excel: {self.result_xlsx_path}')

    def _show_usage_help(self):
        self._show_text_dialog(
            '使用说明',
            """1. 选中设备
   1.1. 点击扫描设备按钮后，在设备预览框中双击选择需要测试的设备。
   1.2. 点击摄像头扫码按钮识别设备二维码，并双击匹配为√的设备。
2. 选中设备后点击连接按钮。
3. 点击开始测试按钮即可开始测试。
4. 测试完成后，在测试结果中确认结果并将测试结果保存。
5. CSV 转 EXCEL 若有 EXCEL 需要则点击即可。""",
        )

    def _show_criteria_help(self):
        self._show_text_dialog(
            '测试结果判据说明',
            """雷达成功判据：
点击雷达开启按钮约 1 秒后，固件上传日志中出现 x=...，y=... 即代表成功，雷达关闭按钮可不使用。

电机成功判据：
点击电机开启按钮后，电机下转 90°、上转 90°、左转 90°、右转 180°、左转 90°，整个流程走完即可代表电机测试成功，电机关闭按钮可不使用。

激光成功判据：
点击激光灯开启或关闭按钮后，激光灯对应开关即可代表测试成功。""",
        )

    def _show_text_dialog(self, title: str, text: str):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(title)
        dialog.resize(760, 520)
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.setContentsMargins(12, 12, 12, 12)
        text_edit = QtWidgets.QPlainTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setPlainText(text)
        text_edit.setStyleSheet(
            'background: #0b1220; color: #e5e7eb; border: 1px solid #334155; border-radius: 4px; padding: 8px;'
        )
        close_btn = QtWidgets.QPushButton('关闭')
        close_btn.clicked.connect(dialog.accept)
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_row.addWidget(close_btn)
        layout.addWidget(text_edit, 1)
        layout.addLayout(btn_row)
        dialog.exec_()

    def _clear_logs(self):
        self.host_log_text.clear()
        self.firmware_log_text.clear()

    def _append_log(self, line: str):
        now = _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.host_log_text.appendPlainText(f'[{now}] {line}')

    def _append_firmware_log(self, line: str):
        now = _dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.firmware_log_text.appendPlainText(f'[{now}] {line}')

    def closeEvent(self, event):
        if self.qr_dialog is not None:
            self.qr_dialog.close()
            self.qr_dialog = None
        self.worker.stop()
        super().closeEvent(event)




