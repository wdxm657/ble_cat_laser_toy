import struct

HEAD = b"\x55\xAA"
VERSION = 0x01
MSG_CMD = 0x01
MSG_RSP = 0x02
MSG_EVT = 0x03

CMD_GPIO_SET = 0x10
CMD_GPIO_ALL_SET = 0x11
CMD_UID_READ = 0x20
CMD_BAT_ADC_READ = 0x21
CMD_NTC_ADC_READ = 0x22
CMD_FW_VERSION_READ = 0x23
CMD_LOW_POWER = 0x30
CMD_STATUS_GET = 0x40

EVT_ADC = 0x80
EVT_KEY = 0x81
EVT_GPIO = 0x82
EVT_UID = 0x83
EVT_USB = 0x84

STATUS_TEXT = {
    0x00: "OK",
    0x01: "LEN_ERROR",
    0x02: "UNSUPPORTED",
    0x03: "PARAM_ERROR",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc


def build_command(cmd_id: int, seq: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BBBBH", VERSION, MSG_CMD, cmd_id, seq, len(payload)) + payload
    return HEAD + body + struct.pack("<H", crc16(body))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEAD)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == HEAD[:1] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            version, msg_type, cmd_id, seq, length = struct.unpack_from("<BBBBH", self.buffer, 2)
            frame_len = 10 + length
            if length > 64:
                del self.buffer[:2]
                continue
            if len(self.buffer) < frame_len:
                break
            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            body = raw[2:-2]
            if version == VERSION and crc16(body) == struct.unpack_from("<H", raw, frame_len - 2)[0]:
                frames.append({
                    "type": msg_type,
                    "cmd": cmd_id,
                    "seq": seq,
                    "payload": raw[8:-2],
                })
        return frames
