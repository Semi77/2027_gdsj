#!/usr/bin/env python3
"""保留原舵机角度接口，通过底盘ROS节点控制STM32 PA8。"""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, String


class ServoPwmController(Node):
    PERIOD_NS = 20_000_000
    MIN_PULSE_NS = 500_000
    MAX_PULSE_NS = 2_500_000
    MIN_ANGLE_DEG = 0.0
    MAX_ANGLE_DEG = 270.0

    def __init__(self):
        """@brief 恢复保存的目标角度，并等待PA8执行接口就绪。"""
        super().__init__('servo_pwm_controller')
        self._state_file = Path('/workspace/ros2_ws/.servo_pwm_state')
        self._feedback = {}
        self._received = self._last_send = self._first_send = 0.0
        self._pending = self._load_saved_angle()
        self.declare_parameter('angle_deg', self._pending)
        self._pending = float(self.get_parameter('angle_deg').value)
        self._validate_angle(self._pending)
        self._publisher = self.create_publisher(String, '/servo/command', 1)
        self.create_subscription(String, '/chassis/feedback', self._on_feedback, 10)
        self.create_subscription(Float64, '/servo/angle_cmd', self._on_angle_topic, 10)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(0.05, self._send_pending)
        self.get_logger().info(f'STM32 PA8 backend: target {self._pending:.2f} deg; waiting for chassis feedback')

    def _load_saved_angle(self):
        """@brief 读取原状态文件，文件不可用时采用原默认角度135度。"""
        try:
            angle = float(self._state_file.read_text(encoding='ascii').strip())
            self._validate_angle(angle)
            return angle
        except (OSError, ValueError):
            return 135.0

    @classmethod
    def _pulse_ns(cls, angle_deg):
        """@brief 保留原线性换算；angle_deg为0至270度目标角度。"""
        cls._validate_angle(angle_deg)
        return round(cls.MIN_PULSE_NS + angle_deg / 270.0 * (cls.MAX_PULSE_NS - cls.MIN_PULSE_NS))

    @classmethod
    def _validate_angle(cls, angle_deg):
        """@brief 校验angle_deg为有限的0至270度角度。"""
        if not math.isfinite(angle_deg) or not cls.MIN_ANGLE_DEG <= angle_deg <= cls.MAX_ANGLE_DEG:
            raise ValueError('angle_deg must be between 0 and 270 degrees')

    def _ready(self):
        """@brief 仅在反馈新鲜、会话一致且脉宽范围已放行时允许发送。"""
        data = self._feedback
        extra = data.get('extra', {})
        return (time.monotonic() - self._received < 0.2 and data.get('wheel_control') and
                data.get('servo_allowed') and extra.get('capabilities', 0) & 2 and
                extra.get('session') == data.get('session') and
                data.get('servo_min_us', 2500) <= 500 and data.get('servo_max_us', 500) >= 2500)

    def _on_feedback(self, msg):
        """@brief 接收底盘反馈，确认PA8比较值后停止重发，不视为实际角度回读。"""
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('extra', {}), dict):
                return
            self._feedback, self._received = data, time.monotonic()
            if self._pending is not None and self._first_send and self._ready():
                pulse = round(self._pulse_ns(self._pending) / 1000)
                if data['extra'].get('servo_pulse_us') == pulse:
                    self.get_logger().info(f'PA8 target confirmed: {self._pending:.2f} deg, {pulse} us')
                    self._pending = None
        except (ValueError, TypeError, KeyError):
            return

    def _write_angle(self, angle_deg, persist):
        """@brief 保存并提交目标；angle_deg为度，persist表示是否更新原状态文件。"""
        self._validate_angle(angle_deg)
        if not self._ready():
            raise RuntimeError('PA8 not ready: open chassis UART3 and check servo limits/unlock')
        if persist:
            temp = self._state_file.with_suffix('.tmp')
            temp.write_text(f'{angle_deg:.6f}\n', encoding='ascii')
            os.replace(temp, self._state_file)
        self._pending = angle_deg
        self._last_send = self._first_send = 0.0

    def _send_pending(self):
        """@brief 重发待确认目标，单次消息有效期由底盘节点限制为200毫秒。"""
        now = time.monotonic()
        if self._pending is None:
            return
        if self._first_send and now - self._first_send > 2.0:
            self.get_logger().error('PA8 command not confirmed; target retained in saved state')
            self._pending = None
            return
        if not self._ready() or now - self._last_send < 0.1:
            return
        pulse = round(self._pulse_ns(self._pending) / 1000)
        self._publisher.publish(String(data=json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds, 'pulse_us': pulse})))
        self._last_send = now
        if not self._first_send:
            self._first_send = now

    def _on_parameters(self, params):
        """@brief 保留angle_deg参数入口，接口未就绪或角度无效时明确拒绝。"""
        for param in params:
            if param.name != 'angle_deg':
                continue
            try:
                if isinstance(param.value, bool):
                    raise ValueError('angle_deg must be numeric')
                self._write_angle(float(param.value), persist=True)
            except (ValueError, TypeError, OSError, RuntimeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _on_angle_topic(self, msg):
        """@brief 保留Float64角度话题，并复用参数校验与保存流程。"""
        result = self.set_parameters([Parameter('angle_deg', value=float(msg.data))])[0]
        if not result.successful:
            self.get_logger().error(f'Ignoring angle command: {result.reason}')


def main(args=None):
    """@brief 运行角度桥接节点，退出后由STM32保持最后PWM。"""
    # 跨容器且用户不同，共享内存不可互通时使用UDP传递ROS消息。
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(Path(__file__).with_name('fastdds_udp.xml')))
    rclpy.init(args=args)
    node = None
    try:
        node = ServoPwmController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
