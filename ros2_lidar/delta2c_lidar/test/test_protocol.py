"""验证协议边界、分块接收、校验恢复和整圈组帧。"""

import math
import struct
import unittest

from delta2c_lidar.protocol import Parser, Point, Packet, ScanBuilder


def frame(start=0, end=22.5, samples=((180, 4001),), version=0x13, speed=False):
    """@brief 构造测试报文；角度单位为度，samples 为强度与原始四分之一毫米距离。"""
    payload = bytes([140])
    if not speed:
        payload += struct.pack(">hHH", 0, round(start * 100), round(end * 100))
        payload += b"".join(struct.pack(">BH", quality, raw) for quality, raw in samples)
    body = b"\xaa" + struct.pack(">HBBBH", 8 + len(payload), version, 0x61,
                                 0xae if speed else 0xad, len(payload)) + payload
    return body + struct.pack(">H", sum(body) & 0xffff)


class ProtocolTest(unittest.TestCase):
    """@brief 覆盖真实协议中的关键边界与失步恢复。"""

    def test_speed_frame_eleven_bytes(self):
        """@brief 确保 11 字节转速帧不被错误的最短长度限制丢弃。"""
        parser = Parser()
        packet, = parser.feed(frame(speed=True))
        self.assertEqual(packet.hz, 7.0)
        self.assertFalse(packet.points)
        self.assertEqual(parser.speed_frames, 1)

    def test_versions_and_quarter_mm(self):
        """@brief 验证两种 2C 版本以及 0.25 毫米的距离精度。"""
        for version in (0x11, 0x13):
            packet, = Parser().feed(frame(version=version))
            self.assertAlmostEqual(packet.points[0].range_m, 1.00025)
        self.assertFalse(Parser().feed(frame(version=1)))

    def test_split_and_checksum_recovery(self):
        """@brief 验证逐字节输入、噪声及坏校验之后仍能恢复下一有效报文。"""
        bad = bytearray(frame())
        bad[-1] ^= 1
        stream = b"noise" + bad + frame(speed=True) + frame()
        parser = Parser()
        packets = []
        for value in stream:
            packets.extend(parser.feed(bytes([value])))
        self.assertEqual(len(packets), 2)
        self.assertEqual(parser.checksum_errors, 1)

    def test_wrap_and_no_echo(self):
        """@brief 验证跨零角插值、无质量、零距离及饱和值过滤。"""
        packet, = Parser().feed(frame(354, 6, ((180, 0), (0, 1000), (180, 16352), (180, 4001))))
        self.assertEqual([point.angle_deg for point in packet.points], [355.5, 358.5, 1.5, 4.5])
        self.assertTrue(all(math.isinf(p.range_m) for p in packet.points[:3]))
        self.assertAlmostEqual(packet.points[-1].range_m, 1.00025)

    def test_malformed_lengths(self):
        """@brief 拒绝数据长度不匹配的帧且不阻塞后续正确帧。"""
        invalid = bytearray(frame())
        invalid[7] += 1
        packets = Parser().feed(invalid + frame())
        self.assertEqual(len(packets), 1)

    def test_complete_scan_and_missing_sectors(self):
        """@brief 丢弃首次半圈，接受完整一圈并拒绝稀疏扫描。"""
        builder = ScanBuilder()
        result = None
        for i in range(73):
            angle = (i % 36) * 10.0
            packet = Packet(0x13, 0xad, 7, angle, (Point(angle, 1.0, 100),))
            result = builder.add(packet, i / 252)
        self.assertIsNotNone(result)
        self.assertEqual(sum(math.isfinite(v) for v in result[0]), 36)
        self.assertEqual(result[2], 1.0)
        self.assertAlmostEqual(result[0][10], 1.0)
        sparse = ScanBuilder()
        for i, angle in enumerate((350, 0, 350, 0)):
            self.assertIsNone(sparse.add(Packet(0x13, 0xad, 7, angle, (Point(angle, 1, 1),)), i / 100))
        self.assertGreater(sparse.dropped, 0)

    def test_long_gap_drops_old_points(self):
        """@brief 断流后不把多圈残片拼成一次正常扫描。"""
        builder = ScanBuilder()
        for angle, stamp in ((350, 0), (0, 0.01), (90, 2.0)):
            self.assertIsNone(builder.add(Packet(0x13, 0xad, 7, angle, (Point(angle, 1, 1),)), stamp))
        self.assertFalse(builder.started)
        self.assertFalse(builder.points)


if __name__ == "__main__":
    unittest.main()
