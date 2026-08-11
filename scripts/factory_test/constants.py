# -*- coding: utf-8 -*-
"""Constants for the W2MLaserTOY factory test tool."""

import os

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
CTRL_FACTORY_TEST_MODULE_RADAR = 0x01
CTRL_FACTORY_TEST_MODULE_MOTOR = 0x02
CTRL_FACTORY_TEST_MODULE_LASER = 0x03

RESULT_HEADERS = [
    "时间",
    "设备名称",
    "MAC/地址",
    "MANUFACTURER_DATA",
    "雷达测试",
    "电机测试",
    "激光灯测试",
    "结果",
]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RESULT_CSV = os.path.join(SCRIPT_DIR, "factory_test_results.csv")
DEFAULT_RESULT_XLSX = os.path.join(SCRIPT_DIR, "factory_test_results.xlsx")

# UI themes: 'dark' = 灰底白字, 'light' = 偏黄白底黑字
THEMES = {
    'dark': {
        'window': '#0f172a',
        'groupbox': '#111827',
        'input': '#0b1220',
        'border': '#334155',
        'text': '#e5e7eb',
        'text_secondary': '#cbd5e1',
        'header_bg': '#1e293b',
        'header_text': '#e2e8f0',
        'button': '#1f2937',
        'button_border': '#475569',
        'button_hover': '#334155',
        'button_pressed': '#475569',
        'button_disabled': '#1e293b',
        'button_disabled_text': '#64748b',
        'alternate': '#111827',
        'selection': '#2563eb',
        'selection_hover': '#1d4ed8',
        'selection_text': '#ffffff',
        'status_bg': '#1e293b',
        'status_text': '#bfdbfe',
        'splitter': '#334155',
        'ok_text': '#86efac',
        'warn_text': '#fbbf24',
        'match_bg': '#164e63',
        'match_text': '#ecfeff',
        'miss_text': '#f87171',
    },
    'light': {
        'window': '#faf5e8',
        'groupbox': '#fdf9ee',
        'input': '#fffdf6',
        'border': '#d6c9a8',
        'text': '#1f2937',
        'text_secondary': '#57534e',
        'header_bg': '#e8dfc8',
        'header_text': '#1f2937',
        'button': '#efe8d6',
        'button_border': '#cbbfa3',
        'button_hover': '#e2d8be',
        'button_pressed': '#d2c5a4',
        'button_disabled': '#f0ead9',
        'button_disabled_text': '#a8a29e',
        'alternate': '#f0e9d6',
        'selection': '#bfdbfe',
        'selection_hover': '#93c5fd',
        'selection_text': '#111827',
        'status_bg': '#e8dfc8',
        'status_text': '#1f2937',
        'splitter': '#c9bca0',
        'ok_text': '#15803d',
        'warn_text': '#b45309',
        'match_bg': '#a7f3d0',
        'match_text': '#064e3b',
        'miss_text': '#dc2626',
    },
}

CTRL_RX_RAW_BYTES = bytes(
    [0x01, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_TX_RAW_BYTES = bytes(
    [0x02, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_LOG_RAW_BYTES = bytes(
    [0x03, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
