"""底盘串口协议：有界缓冲、CRC 校验和显式单位。"""
import binascii
import math
import struct

HEADER = struct.Struct('<2sBBHHI')
COMMAND = struct.Struct('<IBhh')
WHEEL_COMMAND = struct.Struct('<IB4h')
SERVO_COMMAND = struct.Struct('<IBBH')
EXTRA_FEEDBACK = struct.Struct('<II4I4H')
FEEDBACK = struct.Struct('<HHIH4h4h3h3h3hH')
TAG = 0xC201


def pack_frame(kind, sequence, tick, payload):
    """@brief 打包小端协议帧；参数为类型、序号、毫秒时间及负载。"""
    if len(payload) > 80:
        raise ValueError('负载超过 80 字节')
    data = HEADER.pack(b'\xaa\x55', 1, kind, len(payload), sequence & 65535,
                       tick & 0xffffffff) + payload
    return data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff))


class Decoder:
    """按字节流解析分包、粘包与坏包，不信任串口中的长度字段。"""
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        """@brief 输入任意片段，返回已校验的类型、序号、时间和负载列表。"""
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xaa else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, length, seq, tick = HEADER.unpack_from(self.buffer)
            if version != 1 or length > 80:
                del self.buffer[0]
                self.errors += 1
                continue
            total = HEADER.size + length + 2
            if len(self.buffer) < total:
                break
            crc = struct.unpack_from('<H', self.buffer, total - 2)[0]
            if crc != binascii.crc_hqx(self.buffer[2:total-2], 0xffff):
                del self.buffer[0]
                self.errors += 1
                continue
            result.append((kind, seq, tick, bytes(self.buffer[HEADER.size:total-2])))
            del self.buffer[:total]
        return result


def feedback(payload):
    """@brief 将固件反馈转换为 SI 轮速和角度制姿态；原始 IMU 保留原单位。"""
    if len(payload) != FEEDBACK.size:
        raise ValueError('底盘反馈长度错误')
    values = FEEDBACK.unpack(payload)
    tag, flags, session, ack = values[:4]
    if tag not in (TAG, 0xC202):
        raise ValueError('固件标识不匹配')
    return {'firmware': 'wheels-v2' if tag == 0xC202 else 'console-v1',
            'wheel_control': tag == 0xC202, 'armed': bool(flags & 1),
            'imu_valid': bool(flags & 2), 'timeout': bool(flags & 4),
            'session': session, 'ack': ack,
            'speeds': [v / 1000 for v in values[4:8]],
            'targets': [v / 1000 for v in values[8:12]],
            'rpy_deg': [v / 100 for v in values[12:15]],
            'gyro_raw': list(values[15:18]), 'accel_raw': list(values[18:21]),
            'voltage': values[21] / 100}


def finite_number(value, limit):
    """@brief 校验来自外部接口的数值，并拒绝非有限值和越界值。"""
    number = float(value)
    if not math.isfinite(number) or abs(number) > limit:
        raise ValueError('数值无效或超过限制')
    return number


def extra_feedback(payload):
    """@brief 解码累计编码器、能力、故障及舵机输出确认，计数保持uint32原值。"""
    if len(payload) != EXTRA_FEEDBACK.size:
        raise ValueError('扩展反馈长度错误')
    v = EXTRA_FEEDBACK.unpack(payload)
    return {'sample_ms':v[0], 'session':v[1], 'counts':list(v[2:6]),
            'capabilities':v[6], 'faults':v[7], 'servo_pulse_us':v[8], 'servo_ack':v[9]}
