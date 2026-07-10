import struct
from dataclasses import dataclass
from typing import Optional

CTRL_PROTO_VERSION = 0x01

CTRL_MSG_TYPE_CMD = 0x01
CTRL_MSG_TYPE_RSP = 0x02
CTRL_MSG_TYPE_EVENT = 0x03

CTRL_CMD_LED_CTRL = 0x10
CTRL_CMD_LED_QUERY = 0x11
CTRL_CMD_POWER_CTRL = 0x12
CTRL_CMD_STATUS_GET = 0x13

CTRL_CMD_MOTOR_CTRL = 0x20
CTRL_CMD_MOTOR_SET_ZERO = 0x21
CTRL_CMD_MOTOR_DIR_CTRL = 0x22

CTRL_CMD_CFG_SET = 0x30
CTRL_CMD_CFG_GET = 0x31
CTRL_CMD_TIME_SET = 0x32
CTRL_CMD_PLAY_RECORD_GET = 0x33
CTRL_CMD_UID_GET = 0x34
CTRL_CMD_PLAY_RECORD_DELETE = 0x35

CTRL_CMD_TEXT_CHUNK = 0x40

CTRL_CMD_RADAR_RESET_FLASH_CONFIG = 0x56
CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY = 0x57
CTRL_CMD_RADAR_TRACK_SPEED = 0x58
CTRL_CMD_RADAR_CONFIG_SET_HEIGHT = 0x50  # set radar install height
CTRL_CMD_DEVICE_REBOOT = 0x5A

# 狩猎游戏设置 (0x60-0x64)
CTRL_CMD_HUNT_SETTINGS_ENTER     = 0x60
CTRL_CMD_HUNT_SETTINGS_EXIT      = 0x61
CTRL_CMD_HUNT_PREY_RANDOM        = 0x62
CTRL_CMD_HUNT_SETTINGS_SET       = 0x63
CTRL_CMD_HUNT_SETTINGS_GET       = 0x64


@dataclass
class CtrlFrame:
    version: int
    msg_type: int
    cmd_id: int
    seq: int
    payload_len: int
    payload: bytes


def build_ctrl_cmd_frame(cmd_id: int, seq: int, payload: bytes) -> bytes:
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

# int转hex字符串，0-255范围内，前面补0
def int_to_hex(num):
    if num < 0 or num > 255:
        raise ValueError("Input must be an integer between 0 and 255")
    return f"{num:02X}"
    

def parse_ctrl_frame(data: bytes) -> Optional[CtrlFrame]:
    if len(data) < 6:
        return None
    version = data[0]
    msg_type = data[1]
    cmd_id = data[2]
    seq = data[3]
    payload_len = data[4] | (data[5] << 8)
    if len(data) < 6 + payload_len:
        return None
    payload = data[6 : 6 + payload_len]
    # print(data.hex())
    print(f"type:{msg_type} id:{int_to_hex(cmd_id)} payload:{payload.hex()}")
    return CtrlFrame(version, msg_type, cmd_id, seq, payload_len, payload)


def u16le(v: int) -> bytes:
    return struct.pack("<H", int(v) & 0xFFFF)


def s16le(v: int) -> bytes:
    return struct.pack("<h", int(v))

