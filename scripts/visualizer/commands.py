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


def cmd_radar_reset_flash() -> tuple[int, bytes, str]:
    return p.CTRL_CMD_RADAR_RESET_FLASH_CONFIG, b"", "RADAR_RESET_FLASH_CONFIG"


def cmd_device_reboot() -> tuple[int, bytes, str]:
    return p.CTRL_CMD_DEVICE_REBOOT, b"", "DEVICE_REBOOT"


# 新简化配置接口
def cmd_radar_config_set_height(mm: int) -> tuple[int, bytes, str]:
    """新简化配置：设置高度并进入配置模式 (CMD 0x59)"""
    return p.CTRL_CMD_RADAR_CONFIG_SET_HEIGHT, p.s16le(mm), f"RADAR_CONFIG_SET_HEIGHT mm={mm}"


def cmd_radar_config_set_coords_part0(coords: list[tuple[int, int]]) -> tuple[int, bytes, str]:
    """新简化配置：批量设置坐标点 - 第一包 (CMD 0x5B)
    
    发送左上(0)、右上(1)两个坐标点
    
    Args:
        coords: 前2个坐标点列表 [(x0,y0), (x1,y1)]
    
    Returns:
        (cmd_id, payload, description)
    """
    if len(coords) != 2:
        raise ValueError("第一包需要提供2个坐标点（左上、右上）")
    
    payload = bytearray()
    payload.append(0x00)  # partIndex = 0
    for x, y in coords:
        payload.extend(p.s16le(x))
        payload.extend(p.s16le(y))
    
    desc = f"RADAR_CONFIG_SET_COORDS part0: {coords}"
    return p.CTRL_CMD_RADAR_CONFIG_SET_COORDS, bytes(payload), desc


def cmd_radar_config_set_coords_part1(coords: list[tuple[int, int]]) -> tuple[int, bytes, str]:
    """新简化配置：批量设置坐标点 - 第二包 (CMD 0x5B)
    
    发送右下(2)、左下(3)两个坐标点
    
    Args:
        coords: 后2个坐标点列表 [(x2,y2), (x3,y3)]
    
    Returns:
        (cmd_id, payload, description)
    """
    if len(coords) != 2:
        raise ValueError("第二包需要提供2个坐标点（右下、左下）")
    
    payload = bytearray()
    payload.append(0x01)  # partIndex = 1
    for x, y in coords:
        payload.extend(p.s16le(x))
        payload.extend(p.s16le(y))
    
    desc = f"RADAR_CONFIG_SET_COORDS part1: {coords}"
    return p.CTRL_CMD_RADAR_CONFIG_SET_COORDS, bytes(payload), desc


def cmd_radar_config_set_coords(coords: list[tuple[int, int]]) -> list[tuple[int, bytes, str]]:
    """新简化配置：批量设置4个坐标点 (CMD 0x5B) - 分包传输
    
    由于 BLE MTU 限制为 20 字节，需分两包发送。
    
    Args:
        coords: 4个坐标点列表 [(x0,y0), (x1,y1), (x2,y3), (x3,y3)]
                顺序：左上(0) → 右上(1) → 右下(2) → 左下(3)
    
    Returns:
        包含两个命令的列表：[(cmd_id, payload, desc), (cmd_id, payload, desc)]
    """
    if len(coords) != 4:
        raise ValueError("需要提供4个坐标点")
    
    # 第一包：左上、右上
    part0 = cmd_radar_config_set_coords_part0(coords[0:2])
    
    # 第二包：右下、左下
    part1 = cmd_radar_config_set_coords_part1(coords[2:4])
    
    return [part0, part1]
