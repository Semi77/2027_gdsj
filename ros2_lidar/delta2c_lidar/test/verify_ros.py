"""通过伪终端验证真实 ROS 节点的无数据、转速与测距状态，不访问硬件。"""

import math
import os
import pty
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import LaserScan

from delta2c_lidar.node import LidarNode
from test_protocol import frame


def main():
    """@brief 使用伪串口注入已知报文，验证诊断转换和米制扫描内容。"""
    master, slave = pty.openpty()
    rclpy.init(args=["--ros-args", "-p", "port:=" + os.ttyname(slave),
                     "-p", "data_timeout:=0.6", "-r", "/scan:=/lidar_test/scan",
                     "-r", "/diagnostics:=/lidar_test/diagnostics"])
    driver = LidarNode()
    observer = Node("lidar_test_observer")
    scans, statuses = [], []
    observer.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data)
    observer.create_subscription(DiagnosticArray, "/diagnostics", statuses.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(observer)

    def run(duration, source=None):
        """@brief 在 duration 秒内驱动执行器，按 112 帧每秒从 source 生成报文。"""
        deadline = time.monotonic() + duration
        next_write = time.monotonic()
        index = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if source is not None and now >= next_write:
                os.write(master, source(index))
                index += 1
                next_write += 1 / 112
            executor.spin_once(timeout_sec=0.002)

    try:
        run(1.2)
        assert any("无串口数据" in msg.status[0].message for msg in statuses)
        run(1.3, lambda _: frame(speed=True))
        assert any("仅有转速帧" in msg.status[0].message for msg in statuses)
        assert not scans
        run(1.8, lambda i: frame((i % 16) * 22.5, ((i % 16) + 1) * 22.5,
                                 ((180, 4001),) * 15))
        assert len(scans) >= 3, len(scans)
        assert any(msg.status[0].level == DiagnosticStatus.OK for msg in statuses)
        message = scans[-1]
        assert len(message.ranges) == 360
        assert message.header.frame_id == "laser"
        assert message.scan_time > 0
        valid = [value for value in message.ranges if math.isfinite(value)]
        assert len(valid) >= 200
        assert all(abs(value - 1.00025) < 1e-6 for value in valid)
        print(f"PASS: no-data -> speed-only -> valid LaserScan; scans={len(scans)}, valid_bins={len(valid)}")
    finally:
        executor.shutdown()
        observer.destroy_node()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
