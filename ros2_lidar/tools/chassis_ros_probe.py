"""通过 ROS 2 话题测试底盘串口，所有发往底盘的命令均为零速禁能。"""
import argparse
import json
import secrets
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String, UInt32
import serial

from protocol import COMMAND, Decoder, feedback, pack_frame


class ChassisProbe(Node):
    """接收底盘反馈并发布 ROS 状态，只接受禁能探测请求。"""

    def __init__(self, port, bootstrap_disarm=False):
        """@brief 打开 port 指定串口；bootstrap_disarm 仅用于已确认新固件的主动禁能测试。"""
        super().__init__('chassis_serial_probe')
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=.1, exclusive=True)
        self.decoder = Decoder()
        self.publisher = self.create_publisher(String, '/chassis/probe/status', 10)
        self.subscription = self.create_subscription(
            UInt32, '/chassis/probe/disarm', self.on_disarm, 10)
        self.timer = self.create_timer(.01, self.poll)
        self.session = secrets.randbits(32) or 1
        self.sequence = 0
        self.rx_bytes = self.frames = self.tx = self.ack_frames = 0
        self.last_feedback = 0.
        self.first_feedback = None
        self.last_data = None
        self.pending = set()
        self.bootstrap_disarm = bootstrap_disarm

    def on_disarm(self, msg):
        """@brief 收到 ROS 探测请求后只发送禁能帧，msg.data 为探测编号。"""
        if not self.bootstrap_disarm and time.monotonic() - self.last_feedback > .2:
            return
        self.sequence = (self.sequence + 1) & 65535
        payload = COMMAND.pack(self.session, 0, 0, 0)
        packet = pack_frame(1, self.sequence, int(time.monotonic() * 1000), payload)
        if self.serial.write(packet) != len(packet):
            raise OSError('底盘串口发生短写')
        self.pending.add(self.sequence)
        self.tx += 1

    def poll(self):
        """@brief 非阻塞解码底盘反馈，仅将通过 CRC 和固件标识检查的数据发布到 ROS。"""
        raw = self.serial.read(4096)
        self.rx_bytes += len(raw)
        for kind, seq, tick, payload in self.decoder.feed(raw):
            if kind != 0x81:
                continue
            try:
                data = feedback(payload)
            except ValueError:
                continue
            now = time.monotonic()
            self.first_feedback = now if self.first_feedback is None else self.first_feedback
            self.last_feedback = now
            self.frames += 1
            if data['session'] == self.session and data['ack'] in self.pending:
                self.pending.remove(data['ack'])
                self.ack_frames += 1
            self.last_data = data
            msg = String()
            msg.data = json.dumps({'tick_ms': tick, 'sequence': seq, **data})
            self.publisher.publish(msg)


class Observer(Node):
    """通过 ROS 订阅真实反馈，并以 20 Hz 发布固定禁能探测请求。"""

    def __init__(self, bootstrap_disarm=False):
        """@brief 创建 ROS 观察节点；bootstrap_disarm 允许主动发布固定禁能请求。"""
        super().__init__('chassis_probe_observer')
        self.received = 0
        self.requests = 0
        self.bootstrap_disarm = bootstrap_disarm
        self.publisher = self.create_publisher(UInt32, '/chassis/probe/disarm', 10)
        self.subscription = self.create_subscription(
            String, '/chassis/probe/status', self.on_status, 10)
        self.timer = self.create_timer(.05, self.request)

    def on_status(self, msg):
        """@brief 统计从 ROS 收到的有效底盘状态，msg 为 JSON 状态消息。"""
        json.loads(msg.data)
        self.received += 1

    def request(self):
        """@brief 发布固定禁能探测请求，收到真实反馈前不请求串口发送。"""
        if not self.received and not self.bootstrap_disarm:
            return
        self.requests += 1
        msg = UInt32()
        msg.data = self.requests
        self.publisher.publish(msg)


def main():
    """@brief 运行有界通信测试并输出结果，结束时释放串口和所有测试节点。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyAMA3')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--bootstrap-disarm', action='store_true',
                        help='仅在确认已烧入匹配固件后使用，所有帧仍固定为零速禁能')
    args = parser.parse_args()
    rclpy.init(args=[])
    probe = observer = None
    executor = SingleThreadedExecutor()
    try:
        probe = ChassisProbe(args.port, args.bootstrap_disarm)
        observer = Observer(args.bootstrap_disarm)
        executor.add_node(probe)
        executor.add_node(observer)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        result = {
            'port': args.port, 'baud': 115200, 'rx_bytes': probe.rx_bytes,
            'valid_feedback_frames': probe.frames, 'crc_or_header_errors': probe.decoder.errors,
            'ros_received_feedback': observer.received, 'ros_disarm_requests': observer.requests,
            'serial_disarm_frames': probe.tx, 'acknowledged_commands': probe.ack_frames,
            'last_feedback': probe.last_data,
            'passed': bool(probe.frames and observer.received and probe.ack_frames),
            'motion_commands_sent': False,
            'bootstrap_disarm': args.bootstrap_disarm,
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        executor.shutdown()
        if probe is not None:
            probe.serial.close()
            probe.destroy_node()
        if observer is not None:
            observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
