"""使用上游 lds2d 的各型号解码器离线比较同一份雷达串口采样。"""

import argparse
from collections import Counter
import json
import math
import sys
from pathlib import Path


class Replay:
    """@brief 将采样文件作为只读串口回放，禁止任何控制指令。"""

    def __init__(self, data):
        """@brief 初始化回放；data 为原始串口字节。"""
        self.data = data
        self.offset = 0

    def read(self, size):
        """@brief 返回至多 size 个字节，文件结束时停止解码。"""
        if self.offset >= len(self.data):
            raise EOFError
        chunk = self.data[self.offset:self.offset + size]
        self.offset += len(chunk)
        return chunk

    def write(self, data):
        """@brief 拒绝发送 data 中的命令，确保比较过程只处理离线数据。"""
        raise RuntimeError("离线比较不允许发送控制命令")


class NoMotor:
    """@brief 为上游电机接口提供无硬件操作的离线替身。"""

    def set_duty(self, duty):
        """@brief 忽略 duty 占空比，不访问任何 GPIO。"""


def compare(data):
    """@brief 用所有已注册型号解析 data，并返回各型号的匹配统计。"""
    import lds2d
    from lds2d.core import driver_for

    classes = {driver_for(name) for name in lds2d.available_models()}
    results = []
    for cls in sorted(classes, key=lambda item: item.MODEL_NAME):
        kwargs = {"motor": NoMotor()} if cls.NEEDS_MOTOR else {}
        driver = cls(Replay(data), **kwargs)
        # RPLIDAR 离线回放保留响应描述符校验，同时跳过发送启动命令。
        if cls.__module__.endswith("rplidar"):
            driver._started = True
        frames = samples = valid = 0
        distances = []
        error = None
        try:
            for _, points in driver._packets():
                frames += 1
                samples += len(points)
                for point in points:
                    if point.dist_mm > 0 and point.quality > 0:
                        valid += 1
                        distances.append(point.dist_mm)
        except EOFError:
            pass
        except Exception as exc:
            error = str(exc)
        results.append({
            "model": cls.MODEL_NAME, "default_baud": cls.DEFAULT_BAUD,
            "packets": frames, "samples": samples, "valid_points": valid,
            "range_mm": [min(distances), max(distances)] if distances else [],
            "error": error,
        })
    return results


def main():
    """@brief 从命令行读取采样与上游源码路径，输出 JSON 比较结果。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--upstream-src", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream_src.resolve()))
    report = {"capture": str(args.capture), "bytes": args.capture.stat().st_size,
              "models": compare(args.capture.read_bytes())}
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "delta2c_lidar"))
    from delta2c_lidar.protocol import Parser
    decoder = Parser()
    packets = decoder.feed(args.capture.read_bytes())
    frequencies = [packet.hz for packet in packets]
    distances = [point.range_m for packet in packets for point in packet.points
                 if math.isfinite(point.range_m)]
    report["strict_delta2c"] = {
        "frames": decoder.frames, "speed_frames": decoder.speed_frames,
        "measurement_frames": decoder.measurements,
        "checksum_errors": decoder.checksum_errors,
        "versions": dict(Counter(hex(packet.version) for packet in packets)),
        "hz_min_max": [min(frequencies), max(frequencies)] if frequencies else [],
        "valid_points": len(distances),
        "range_m": [min(distances), max(distances)] if distances else [],
    }
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
