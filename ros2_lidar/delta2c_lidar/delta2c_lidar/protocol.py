"""按 DELTA-2C PRO 协议解析串口流，保留四分之一毫米的测距精度。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point:
    """@brief 保存传感器原始方向的角度、米制距离和回波强度。"""

    angle_deg: float
    range_m: float
    quality: int


@dataclass(frozen=True)
class Packet:
    """@brief 保存协议版本、报文类型、转速、起始角及采样点。"""

    version: int
    kind: int
    hz: float
    start_deg: float
    points: tuple


class Parser:
    """@brief 解析 0x11/0x13 版本的测距帧和转速帧并统计异常。"""

    def __init__(self):
        """@brief 初始化流缓冲区和累计接收统计。"""
        self.buffer = bytearray()
        self.received = self.frames = self.checksum_errors = 0
        self.measurements = self.speed_frames = self.rejected = 0

    def feed(self, data):
        """@brief 消费任意分块的 data 串口字节，返回完整且校验正确的报文。"""
        self.received += len(data)
        self.buffer.extend(data)
        packets = []
        while self.buffer:
            start = self.buffer.find(b"\xaa")
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 8:
                break
            buf = self.buffer
            length = int.from_bytes(buf[1:3], "big")
            dlen = int.from_bytes(buf[6:8], "big")
            kind = buf[5]
            shape_ok = (kind == 0xae and dlen == 1) or (
                kind == 0xad and dlen >= 10 and (dlen - 7) % 3 == 0)
            if (buf[3] not in (0x11, 0x13) or buf[4] != 0x61
                    or not 9 <= length <= 512 or length != 8 + dlen or not shape_ok):
                self.rejected += 1
                del buf[0]
                continue
            if len(buf) < length + 2:
                break
            frame = bytes(buf[:length + 2])
            if sum(frame[:-2]) & 0xffff != int.from_bytes(frame[-2:], "big"):
                self.checksum_errors += 1
                del buf[0]
                continue
            del buf[:length + 2]
            points = []
            begin = 0.0
            if kind == 0xad:
                begin = int.from_bytes(frame[11:13], "big") / 100.0
                end = int.from_bytes(frame[13:15], "big") / 100.0
                if begin >= 360 or end > 360 or (end - begin) % 360 > 90:
                    self.rejected += 1
                    continue
                count = (dlen - 7) // 3
                span = (end - begin) % 360
                for i in range(count):
                    offset = 15 + 3 * i
                    quality = frame[offset]
                    raw = int.from_bytes(frame[offset + 1:offset + 3], "big")
                    distance = raw * 0.00025
                    if quality == 0 or raw == 0 or raw > 16000:
                        distance = math.inf
                    angle = (begin + span * (i + 0.5) / count) % 360
                    points.append(Point(angle, distance, quality))
                self.measurements += 1
            else:
                self.speed_frames += 1
            self.frames += 1
            packets.append(Packet(frame[3], kind, frame[8] / 20.0, begin, tuple(points)))
        return packets


class ScanBuilder:
    """@brief 按角度回绕组帧，丢弃首次半圈和长时间断流后的残缺扫描。"""

    def __init__(self, bins=360, clockwise=True, angle_offset=0.0, min_coverage=0.8):
        """@brief bins 为角度槽数，clockwise 为顺时针安装，angle_offset 为度偏置，min_coverage 为覆盖率下限。"""
        self.bins = bins
        self.clockwise = clockwise
        self.angle_offset = angle_offset
        self.min_coverage = min_coverage
        self.previous = None
        self.started = False
        self.start_time = 0.0
        self.points = []
        self.dropped = 0

    def add(self, packet, now):
        """@brief 接收 packet 和单调时间 now，返回完整扫描的距离、强度、覆盖率与周期。"""
        if not packet.points:
            return None
        result = None
        if self.started and now - self.start_time > max(0.5, 2 / max(packet.hz, 1.0)):
            self.started = False
            self.points.clear()
            self.dropped += 1
        wrapped = self.previous is not None and packet.start_deg + 180 < self.previous
        if wrapped:
            if self.started:
                result = self._finish(now - self.start_time)
            self.points = []
            self.started = True
            self.start_time = now
        self.previous = packet.start_deg
        if self.started:
            self.points.extend(packet.points)
        return result

    def _finish(self, period):
        """@brief 将一圈数据投到等角度槽，period 为实际采集周期，缺测槽填正无穷。"""
        ranges = [math.inf] * self.bins
        intensities = [0.0] * self.bins
        sectors = set()
        for point in self.points:
            sectors.add(int(point.angle_deg // 10) % 36)
            angle = ((-point.angle_deg if self.clockwise else point.angle_deg)
                     + self.angle_offset) % 360
            index = int(angle * self.bins / 360 + 0.5) % self.bins
            if point.range_m < ranges[index]:
                ranges[index] = point.range_m
                intensities[index] = float(point.quality)
        coverage = len(sectors) / 36
        if coverage < self.min_coverage:
            self.dropped += 1
            return None
        return ranges, intensities, coverage, period
