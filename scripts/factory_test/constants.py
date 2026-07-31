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

CTRL_RX_RAW_BYTES = bytes(
    [0x01, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_TX_RAW_BYTES = bytes(
    [0x02, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
CTRL_LOG_RAW_BYTES = bytes(
    [0x03, 0xA0, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00]
)
