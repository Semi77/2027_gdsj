"""接收雷达串口，发布标准 LaserScan 与分级诊断信息。"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
import serial

from .protocol import Parser, ScanBuilder


class LidarNode(Node):
    """@brief 管理只读雷达串口、断线重连、整圈发布及故障诊断。"""

    def __init__(self):
        """@brief 读取串口与扫描参数，并启动接收和诊断定时器。"""
        super().__init__("delta2c_lidar")
        defaults = {"port": "/dev/ttyAMA0", "baudrate": 115200,
                    "frame_id": "laser", "scan_bins": 360,
                    "clockwise": True, "angle_offset_deg": 0.0,
                    "min_coverage": 0.8, "range_min": 0.05,
                    "range_max": 4.0, "data_timeout": 3.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        cfg = self.settings
        if not 36 <= cfg["scan_bins"] <= 3600:
            raise ValueError("scan_bins 必须在 36 到 3600 之间")
        if not 0 < cfg["min_coverage"] <= 1 or not 0 < cfg["range_min"] < cfg["range_max"] <= 4:
            raise ValueError("覆盖率应在 (0,1] 内，距离需满足 0 < range_min < range_max <= 4 米")
        if cfg["data_timeout"] <= 0 or cfg["baudrate"] <= 0:
            raise ValueError("data_timeout 和 baudrate 必须为正数")
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.port = None
        self.parser = Parser()
        self.builder = self._new_builder()
        self.next_open = 0.0
        self.opened = self.last_byte = self.last_packet = 0.0
        self.last_measurement = self.last_scan = 0.0
        self.hz = self.coverage = 0.0
        self.version = self.valid_ranges = self.scans = 0
        self.last_state = None
        self.io_error = ""
        self.create_timer(0.01, self._receive)
        self.create_timer(1.0, self._diagnose)
        self.get_logger().info(f"监听 {cfg['port']}，{cfg['baudrate']} 8N1；不发送串口控制命令")

    def _new_builder(self):
        """@brief 根据启动参数创建新的扫描组帧器，避免跨连接拼接数据。"""
        cfg = self.settings
        return ScanBuilder(cfg["scan_bins"], cfg["clockwise"],
                           cfg["angle_offset_deg"], cfg["min_coverage"])

    def _receive(self):
        """@brief 非阻塞接收串口字节，端口失效时释放资源并延迟重连。"""
        now = time.monotonic()
        try:
            if self.port is None:
                if now < self.next_open:
                    return
                self.port = serial.Serial(self.settings["port"], self.settings["baudrate"],
                                          timeout=0, exclusive=True)
                self.opened = now
                self.last_byte = self.last_packet = self.last_measurement = self.last_scan = 0.0
                self.parser.buffer.clear()
                self.builder = self._new_builder()
                self.io_error = ""
                self.get_logger().info("串口已打开，等待雷达数据")
            size = self.port.in_waiting
            if size == 0:
                return
            data = self.port.read(min(size, 8192))
            if data:
                self.last_byte = now
            for packet in self.parser.feed(data):
                self.last_packet = now
                self.hz = packet.hz
                self.version = packet.version
                if packet.points:
                    self.last_measurement = now
                scan = self.builder.add(packet, now)
                if scan is not None:
                    self._publish(scan, now)
        except (serial.SerialException, OSError) as exc:
            self.io_error = str(exc)
            if self.port is not None:
                self.port.close()
                self.port = None
            self.next_open = now + 2.0

    def _publish(self, scan, now):
        """@brief 发布 scan 的等角度距离数组，now 用于记录最近有效扫描时间。"""
        ranges, intensities, self.coverage, period = scan
        cfg = self.settings
        msg = LaserScan()
        stamp = self.get_clock().now().nanoseconds - int(period * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(max(0, stamp), 1_000_000_000)
        msg.header.frame_id = cfg["frame_id"]
        msg.angle_min = 0.0
        msg.angle_increment = math.tau / cfg["scan_bins"]
        msg.angle_max = (cfg["scan_bins"] - 1) * msg.angle_increment
        msg.scan_time = period
        # 雷达没有逐点时间戳，角度重排后不虚构点间采样时间。
        msg.time_increment = 0.0
        msg.range_min = cfg["range_min"]
        msg.range_max = cfg["range_max"]
        msg.ranges = [distance if cfg["range_min"] <= distance <= cfg["range_max"]
                      else math.inf for distance in ranges]
        msg.intensities = intensities
        self.valid_ranges = sum(math.isfinite(value) for value in msg.ranges)
        self.scan_pub.publish(msg)
        self.last_scan = now
        self.scans += 1

    def _diagnose(self):
        """@brief 区分串口故障、无信号、协议错误、仅转速、扫描残缺和正常测距。"""
        now = time.monotonic()
        timeout = self.settings["data_timeout"]
        if self.port is None:
            level, message = DiagnosticStatus.ERROR, "串口不可用：" + self.io_error
        elif now - self.opened < timeout and not self.last_packet:
            level, message = DiagnosticStatus.WARN, "正在等待雷达数据"
        elif now - self.last_byte > timeout:
            level, message = DiagnosticStatus.ERROR, "无串口数据：检查 TX、共地及雷达供电"
        elif now - self.last_packet > timeout:
            level, message = DiagnosticStatus.ERROR, "收到字节但无有效 2C 帧：检查型号、波特率或接线干扰"
        elif now - self.last_measurement > timeout:
            level, message = DiagnosticStatus.WARN, "仅有转速帧：检查电机供电与转速窗口"
        elif now - self.last_scan > timeout:
            level, message = DiagnosticStatus.WARN, "测距帧不连续，尚未形成覆盖率合格的一圈扫描"
        elif not self.valid_ranges:
            level, message = DiagnosticStatus.WARN, "扫描已成帧，但没有有效距离回波"
        else:
            level, message = DiagnosticStatus.OK, "雷达测距与整圈扫描正常"
        status = DiagnosticStatus(level=level, name="delta2c_lidar/serial",
                                  message=message, hardware_id="DELTA2C Pro-D-V001")
        values = {"port": self.settings["port"], "bytes": self.parser.received,
                  "protocol_version": hex(self.version), "scan_hz": round(self.hz, 2),
                  "valid_frames": self.parser.frames, "speed_frames": self.parser.speed_frames,
                  "measurement_frames": self.parser.measurements,
                  "checksum_errors": self.parser.checksum_errors,
                  "rejected_headers": self.parser.rejected, "published_scans": self.scans,
                  "dropped_scans": self.builder.dropped, "coverage": round(self.coverage, 3),
                  "valid_ranges": self.valid_ranges}
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message_array = DiagnosticArray(status=[status])
        message_array.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(message_array)
        state = (level, message)
        if state != self.last_state:
            text = f"{message}；转速 {self.hz:.2f} Hz，累计测距帧 {self.parser.measurements}"
            if level == DiagnosticStatus.OK:
                self.get_logger().info(text)
            else:
                self.get_logger().warning(text)
            self.last_state = state

    def destroy_node(self):
        """@brief 关闭串口并释放 ROS 节点资源。"""
        if self.port is not None:
            self.port.close()
            self.port = None
        return super().destroy_node()


def main(args=None):
    """@brief 初始化并运行雷达节点，退出时释放串口资源。"""
    rclpy.init(args=args)
    node = None
    try:
        node = LidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
