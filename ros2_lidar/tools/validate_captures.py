"""将真实采样与上游测距解码器交叉验证，防止距离比例和字节序回归。"""

import argparse
import json
import math
from pathlib import Path
import re
import sys


def main():
    """@brief 校验采样目录中的日志，输出逐文件有效帧数与交叉比较结果。"""
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("samples", type=Path)
    args_parser.add_argument("--upstream-src", type=Path, required=True)
    args = args_parser.parse_args()
    sys.path.insert(0, str(args.upstream_src.resolve()))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "delta2c_lidar"))
    from lds2d.drivers.threeirobotix_delta_2c import decode_delta_2c
    from delta2c_lidar.protocol import Parser

    for log in sorted(args.samples.glob("*_raw.log")):
        lines = [line for line in log.read_text().splitlines()
                 if re.fullmatch(r"\s*(?:[0-9A-Fa-f]{2}\s+)*[0-9A-Fa-f]{2}\s*", line)]
        raw = bytes.fromhex(" ".join(lines))
        parser = Parser()
        packets = []
        for offset in range(0, len(raw), 37):
            packets.extend(parser.feed(raw[offset:offset + 37]))
        actual = [packet for packet in packets if packet.points]
        reference = [points for _, points in decode_delta_2c(bytearray(raw)) if points]
        assert len(actual) == len(reference), (log.name, len(actual), len(reference))
        if log.name == "delta2c_20260820_164235_raw.log":
            assert len(actual) == 618
        for packet, points in zip(actual, reference):
            assert len(packet.points) == len(points)
            for ours, upstream in zip(packet.points, points):
                assert abs(ours.angle_deg - upstream.angle_deg) < 1e-6
                if math.isfinite(ours.range_m):
                    assert abs(ours.range_m * 1000 - upstream.dist_mm) <= 0.500001, (
                        log.name, ours.range_m, upstream.dist_mm)
                assert ours.quality == upstream.quality
        print(json.dumps({"sample": log.name, "measurement_frames": len(actual),
                          "speed_frames": parser.speed_frames,
                          "checksum_errors": parser.checksum_errors, "cross_check": "PASS"}))


if __name__ == "__main__":
    main()
