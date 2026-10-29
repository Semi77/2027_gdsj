"""离线验证角度映射、就绪门控、目标保存与PA8反馈确认。"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch


def load_bridge():
    """@brief 仅替代ROS导入，运行实际桥接类的方法。"""
    modules = {name: types.ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.parameter', 'rcl_interfaces',
        'rcl_interfaces.msg', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.node'].Node = object
    modules['rclpy.parameter'].Parameter = object
    modules['rcl_interfaces.msg'].SetParametersResult = types.SimpleNamespace
    modules['std_msgs.msg'].Float64 = types.SimpleNamespace
    modules['std_msgs.msg'].String = types.SimpleNamespace
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('pa8_bridge', Path(__file__).parent / 'servo_pwm_controller/servo_pwm_node.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module.ServoPwmController


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 加载真实角度桥接实现。"""
        cls.bridge = load_bridge()

    def setUp(self):
        """@brief 创建独立临时角度存储和消息收集器。"""
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.node = self.bridge.__new__(self.bridge)
        self.node._state_file = Path(self.directory.name) / '.servo_pwm_state'
        self.node._feedback = {'wheel_control': True, 'servo_allowed': True,
            'servo_min_us': 500, 'servo_max_us': 2500, 'session': 7,
            'extra': {'session': 7, 'capabilities': 2}}
        self.node._received = time.monotonic()
        self.node._pending = None
        self.node._last_send = self.node._first_send = 0.0
        self.messages = []
        self.node._publisher = types.SimpleNamespace(publish=self.messages.append)
        self.node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=123456789))
        self.node.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, error=lambda *a: None)

    def test_mapping(self):
        """@brief 核验原270度映射和异常角度拒绝。"""
        for angle, pulse in [(0,500000),(135,1500000),(270,2500000),(136,1507407)]:
            self.assertEqual(self.bridge._pulse_ns(angle),pulse)
        for angle in (-1,271,float('nan'),float('inf')):
            with self.assertRaises(ValueError): self.bridge._pulse_ns(angle)

    def test_send_save_and_confirm(self):
        """@brief 验证保存格式不变、发送整数微秒并等待反馈确认。"""
        self.node._write_angle(136, True)
        self.assertEqual(self.node._load_saved_angle(),136)
        self.node._send_pending()
        self.assertEqual(json.loads(self.messages[0].data),{'stamp_ns':123456789,'pulse_us':1507})
        data = self.node._feedback
        data['extra']['servo_pulse_us'] = 1507
        self.node._on_feedback(types.SimpleNamespace(data=json.dumps(data)))
        self.assertIsNone(self.node._pending)

    def test_stale_locked_or_wrong_session(self):
        """@brief 拒绝陈旧反馈、软件锁定及会话不匹配，避免离线修改目标。"""
        for key,value in [('servo_allowed',False),('session',8),('servo_min_us',1000)]:
            original=self.node._feedback[key];self.node._feedback[key]=value
            with self.assertRaises(RuntimeError):self.node._write_angle(136,True)
            self.node._feedback[key]=original
        self.node._received=time.monotonic()-1
        self.node._pending=136
        self.node._send_pending()
        self.assertEqual(self.messages,[])


if __name__ == '__main__':
    unittest.main()
