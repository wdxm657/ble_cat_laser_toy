# -*- coding: utf-8 -*-
"""Camera QR scanning dialog for factory test."""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from .constants import THEMES
from .deps import CV2_IMPORT_ERROR, PYZBAR_IMPORT_ERROR, cv2, decode_qr
from .protocol import mfr_matches

_decode_qr = decode_qr
_mfr_matches = mfr_matches
MATCH_MARK = '√'
MISS_MARK = '×'

class CameraQrScanDialog(QtWidgets.QDialog):
    device_matched = QtCore.pyqtSignal(str, str)

    def __init__(self, parent=None, theme: str = 'dark'):
        super().__init__(parent)
        self.theme = theme
        self.setWindowTitle('二维码扫描选择设备')
        self.resize(1120, 680)
        self.devices = []
        self.last_qr_value = ''
        self.matched_address = ''
        self._cap = None
        self._cv_qr_detector = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_camera_tick)
        self._build_ui()
        self._start_camera()

    def _build_ui(self):
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        left_box = QtWidgets.QGroupBox('扫描到的蓝牙设备')
        left_layout = QtWidgets.QVBoxLayout(left_box)
        self.qr_label = QtWidgets.QLabel('二维码: 未识别')
        self.qr_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        left_layout.addWidget(self.qr_label)

        self.device_table = QtWidgets.QTableWidget()
        self.device_table.setColumnCount(5)
        self.device_table.setHorizontalHeaderLabels(['名称', 'MAC/地址', 'RSSI', '匹配', 'MANUFACTURER_DATA'])
        self.device_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.device_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.device_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.device_table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.device_table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.device_table.setAlternatingRowColors(True)
        self.device_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.device_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.device_table.cellDoubleClicked.connect(self._on_device_row_double_clicked)
        self.device_table.itemDoubleClicked.connect(self._on_device_item_double_clicked)
        left_layout.addWidget(self.device_table)

        right_box = QtWidgets.QGroupBox('摄像头实时画面')
        right_layout = QtWidgets.QVBoxLayout(right_box)
        self.preview_label = QtWidgets.QLabel('正在打开摄像头...')
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(520, 390)
        right_layout.addWidget(self.preview_label, 1)

        layout.addWidget(left_box, 1)
        layout.addWidget(right_box, 1)
        self._apply_theme()

    def _palette(self):
        return THEMES.get(self.theme, THEMES['dark'])

    def set_theme(self, theme: str):
        self.theme = theme
        self._apply_theme()
        self._refresh_device_table()

    def _apply_theme(self):
        c = self._palette()
        self.device_table.setStyleSheet(f"""
            QTableWidget {{
                background: {c['input']};
                alternate-background-color: {c['alternate']};
                color: {c['text']};
                gridline-color: {c['border']};
                selection-background-color: {c['selection_hover']};
                selection-color: {c['selection_text']};
            }}
            QTableWidget::item {{
                background: {c['input']};
                color: {c['text']};
            }}
            QTableWidget::item:alternate {{
                background: {c['alternate']};
            }}
            QHeaderView::section {{
                background: {c['header_bg']};
                color: {c['header_text']};
                border: 1px solid {c['border']};
                padding: 6px 8px;
                font-weight: 600;
            }}
        """)
        self.preview_label.setStyleSheet(
            f'background: {c["input"]}; border: 1px solid {c["border"]}; color: {c["text_secondary"]};'
        )

    def _start_camera(self):
        if cv2 is None:
            self.preview_label.setText(f'未安装 opencv-python: {CV2_IMPORT_ERROR}')
            return
        try:
            self._cv_qr_detector = cv2.QRCodeDetector()
        except Exception:
            self._cv_qr_detector = None
        if _decode_qr is None:
            self.qr_label.setText('二维码: 未识别（pyzbar/zbar 不可用，使用 OpenCV 识别）')
        self._cap = cv2.VideoCapture(0)
        if not self._cap or not self._cap.isOpened():
            self.preview_label.setText('摄像头打开失败')
            return
        self._timer.start(40)

    def update_devices(self, devices):
        self.devices = list(devices)
        if self.last_qr_value:
            self._try_match_current_qr()
        else:
            self._refresh_device_table()

    def _refresh_device_table(self, matched_address: str = ''):
        if matched_address:
            self.matched_address = matched_address
        matched_address = matched_address or self.matched_address
        self.device_table.setRowCount(len(self.devices))
        mono_font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        for row_idx, dev in enumerate(self.devices):
            matched = MATCH_MARK if matched_address and dev.address == matched_address else MISS_MARK
            values = [
                dev.name,
                dev.address,
                '' if dev.rssi is None else str(dev.rssi),
                matched,
                dev.manufacturer_data,
            ]
            c = self._palette()
            for col_idx, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setBackground(QtGui.QColor(c['input'] if row_idx % 2 == 0 else c['alternate']))
                item.setForeground(QtGui.QColor(c['text']))
                if col_idx in (1, 4):
                    item.setFont(mono_font)
                if col_idx in (2, 3):
                    item.setTextAlignment(Qt.AlignCenter)
                else:
                    item.setTextAlignment(Qt.AlignVCenter | Qt.AlignLeft)
                if matched == MATCH_MARK:
                    item.setBackground(QtGui.QColor(c['match_bg']))
                    item.setForeground(QtGui.QColor(c['match_text']))
                elif col_idx == 3:
                    item.setForeground(QtGui.QColor(c['miss_text']))
                self.device_table.setItem(row_idx, col_idx, item)
        self.device_table.resizeRowsToContents()

    def _on_camera_tick(self):
        if not self._cap:
            return
        ok, frame = self._cap.read()
        if not ok:
            return
        self._decode_frame(frame)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        image = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(image)
        self.preview_label.setPixmap(
            pixmap.scaled(self.preview_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def _decode_frame(self, frame):
        value = ''
        if _decode_qr is not None:
            try:
                decoded = _decode_qr(frame)
            except Exception:
                decoded = []
            if decoded:
                raw = decoded[0].data
                try:
                    value = raw.decode('utf-8').strip()
                except Exception:
                    value = raw.hex().upper()
        if not value and self._cv_qr_detector is not None:
            value = self._decode_frame_by_opencv(frame)
        if value and value != self.last_qr_value:
            self.last_qr_value = value
            self.qr_label.setText(f'二维码: {value}')
            self._try_match_current_qr()

    def _decode_frame_by_opencv(self, frame):
        try:
            value, _points, _straight = self._cv_qr_detector.detectAndDecode(frame)
        except Exception:
            return ''
        return (value or '').strip()

    def _try_match_current_qr(self):
        if not self.last_qr_value:
            return
        for dev in self.devices:
            if _mfr_matches(self.last_qr_value, dev.manufacturer_data):
                if self.matched_address != dev.address:
                    self.matched_address = dev.address
                    self.device_matched.emit(dev.address, self.last_qr_value)
                self._refresh_device_table(dev.address)
                return
        self.matched_address = ''
        self._refresh_device_table()

    def _on_device_row_double_clicked(self, row: int, _column: int):
        self._select_matched_row(row)

    def _on_device_item_double_clicked(self, item):
        self._select_matched_row(item.row())

    def _select_matched_row(self, row: int):
        if row < 0 or row >= len(self.devices):
            return
        dev = self.devices[row]
        if not self.last_qr_value:
            self.qr_label.setText('二维码: 未识别，无法匹配设备')
            return
        if not _mfr_matches(self.last_qr_value, dev.manufacturer_data):
            self.qr_label.setText(f'二维码已识别，但该行 MANUFACTURER_DATA 不匹配: {self.last_qr_value}')
            return
        self.matched_address = dev.address
        self._refresh_device_table(dev.address)
        self.device_matched.emit(dev.address, self.last_qr_value)
        self.accept()

    def closeEvent(self, event):
        self._timer.stop()
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        super().closeEvent(event)



