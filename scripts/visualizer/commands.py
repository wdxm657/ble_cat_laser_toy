import struct

from . import protocol as p


def cmd_power_ctrl(on: int) -> tuple[int, bytes, str]:
    return p.CTRL_CMD_POWER_CTRL, bytes([1 if on else 0]), f"POWER_CTRL on={on}"


def cmd_motor_dir_start(direction: int, speed_lv: int = 2) -> tuple[int, bytes, str]:
    # payload: op=0x01(move), direction, speed
    return (
        p.CTRL_CMD_MOTOR_DIR_CTRL,
        bytes([0x01, direction & 0xFF, speed_lv & 0xFF]),
        f"MOTOR_DIR start dir={direction} speed={speed_lv}",
    )


def cmd_motor_dir_stop(direction: int = 0, speed_lv: int = 2) -> tuple[int, bytes, str]:
    # payload: op=0x00(stop), direction/speed are ignored by FW but keep frame shape
    return (
        p.CTRL_CMD_MOTOR_DIR_CTRL,
        bytes([0x00, direction & 0xFF, speed_lv & 0xFF]),
        "MOTOR_DIR stop",
    )


def cmd_time_set(epoch_sec: int, tz_q15: int) -> tuple[int, bytes, str]:
    """TIME_SET (0x32): payload = u32 epoch seconds (LE) + s8 tz_q15."""
    ep = int(epoch_sec) & 0xFFFFFFFF
    tz = int(tz_q15)
    if tz < -128:
        tz = -128
    if tz > 127:
        tz = 127
    payload = struct.pack("<Ib", ep, tz)
    return p.CTRL_CMD_TIME_SET, payload, f"TIME_SET epoch={ep} tz_q15={tz}"


def cmd_motor_dir(op: int, direction: int, speed_lv: int) -> tuple[int, bytes, str]:
    return p.CTRL_CMD_MOTOR_DIR_CTRL, bytes([op & 0xFF, direction & 0xFF, speed_lv & 0xFF]), f"MOTOR_DIR op={op} dir={direction} speed={speed_lv}"


def cmd_play_record_ack() -> tuple[int, bytes, str]:
    """PLAY_RECORD_GET ACK (CMD 0x33): 告知设备已收到当前记录，请求发送下一条。
    
    设备行为：收到此 ACK 后，若还有未上传的记录，会在 1 秒后发送下一条 EVENT。
    """
    return p.CTRL_CMD_PLAY_RECORD_GET, b"", "PLAY_RECORD_ACK"


def cmd_radar_reset_flash() -> tuple[int, bytes, str]:
    return p.CTRL_CMD_RADAR_RESET_FLASH_CONFIG, b"", "RADAR_RESET_FLASH_CONFIG"


def cmd_device_reboot() -> tuple[int, bytes, str]:
    return p.CTRL_CMD_DEVICE_REBOOT, b"", "DEVICE_REBOOT"


def cmd_radar_config_set_height(mm: int) -> tuple[int, bytes, str]:
    """设置雷达安装高度 (CMD 0x50)"""
    return p.CTRL_CMD_RADAR_CONFIG_SET_HEIGHT, p.s16le(mm), f"RADAR_CONFIG_SET_HEIGHT mm={mm}"


# ===== 狩猎游戏设置命令 (0x60-0x66) =====

def cmd_hunt_settings_enter() -> tuple[int, bytes, str]:
    """进入狩猎设置模式 (0x60)"""
    return p.CTRL_CMD_HUNT_SETTINGS_ENTER, b"", "HUNT_SETTINGS_ENTER"


def cmd_hunt_settings_exit(apply: bool = True) -> tuple[int, bytes, str]:
    """退出狩猎设置模式 (0x61)"""
    return p.CTRL_CMD_HUNT_SETTINGS_EXIT, bytes([1 if apply else 0]), f"HUNT_SETTINGS_EXIT apply={apply}"


def cmd_hunt_prey_random(start: bool = True) -> tuple[int, bytes, str]:
    """猎物点随机移动 (0x62) start=True 开始, False 停止"""
    return p.CTRL_CMD_HUNT_PREY_RANDOM, bytes([1 if start else 0]), f"HUNT_PREY_RANDOM start={start}"


def cmd_hunt_prey_set() -> tuple[int, bytes, str]:
    """当前云台位置设为猎物点 (0x63)"""
    return p.CTRL_CMD_HUNT_PREY_SET, b"", "HUNT_PREY_SET"


def cmd_hunt_set_duration(seconds: int) -> tuple[int, bytes, str]:
    """设置单次狩猎时长 (0x64), u16 LE 秒"""
    return p.CTRL_CMD_HUNT_SET_DURATION, p.u16le(seconds), f"HUNT_SET_DURATION s={seconds}"


def cmd_hunt_set_count(count: int) -> tuple[int, bytes, str]:
    """设置狩猎次数 (0x65), u8"""
    return p.CTRL_CMD_HUNT_SET_COUNT, bytes([count & 0xFF]), f"HUNT_SET_COUNT cnt={count}"


def cmd_hunt_set_sleep_duration(minutes: int) -> tuple[int, bytes, str]:
    """设置休眠时长 (0x66), u8 分钟"""
    return p.CTRL_CMD_HUNT_SET_SLEEP_DURATION, bytes([minutes & 0xFF]), f"HUNT_SET_SLEEP_DURATION min={minutes}"
