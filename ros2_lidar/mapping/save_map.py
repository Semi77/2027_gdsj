"""订阅真实 OccupancyGrid 并保存 PGM、YAML 与统计信息。"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


def main():
    """@brief output 为输出文件名前缀；接收有效 /map 后保存地图，超时明确报错。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("lidar_map_snapshot")
    maps = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", maps.append, qos)
    deadline = time.monotonic() + 20
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if maps and any(v >= 65 for v in maps[-1].data) and any(0 <= v <= 25 for v in maps[-1].data):
                break
        if not maps:
            raise RuntimeError("20 秒内未收到 /map，尚不能判定建图成功")
        msg = maps[-1]
        width, height = msg.info.width, msg.info.height
        occupied = sum(value >= 65 for value in msg.data)
        free = sum(0 <= value <= 25 for value in msg.data)
        if not occupied or not free:
            raise RuntimeError("地图缺少障碍或空闲栅格，暂不保存为有效地图")
        pixels = bytearray()
        for row in range(height - 1, -1, -1):
            for value in msg.data[row * width:(row + 1) * width]:
                pixels.append(205 if value < 0 else (0 if value >= 65 else (254 if value <= 25 else 205)))
        pgm = args.output.with_suffix(".pgm")
        pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
        origin = msg.info.origin.position
        rotation = msg.info.origin.orientation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        args.output.with_suffix(".yaml").write_text(
            f"image: {pgm.name}\nmode: trinary\nresolution: {msg.info.resolution}\n"
            f"origin: [{origin.x}, {origin.y}, {yaw}]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
        report = {"frame": msg.header.frame_id, "width": width, "height": height,
                  "resolution": msg.info.resolution, "occupied_cells": occupied,
                  "free_cells": free, "unknown_cells": sum(v < 0 for v in msg.data),
                  "pgm": str(pgm)}
        args.output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
