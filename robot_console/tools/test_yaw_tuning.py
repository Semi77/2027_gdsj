"""调参请求、真实参数校验回调与原子保存的离线测试，不访问树莓派。"""
import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_pi_control import load_node_without_ros
from pi.parameter_store import save_parameters
from pi.heading_control import HeadingControl
from pi.wheel_odometry import WheelOdometry


class TuningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 加载真实控制节点回调和网关类，替代ROS导入而不替代校验逻辑。"""
        cls.node_type=load_node_without_ros()
        with patch.dict(sys.modules,{'fcntl':types.ModuleType('fcntl')}):
            cls.gateway_module=importlib.import_module('pi.gateway')

    def setUp(self):
        """@brief 构造内存参数节点，保留原有非PID配置用于验证保存完整性。"""
        self.node=self.node_type.__new__(self.node_type)
        self.node.control=HeadingControl()
        self.node.odometry=WheelOdometry()
        defaults=self.node_type.__init__.__globals__['FEATURE_DEFAULTS'].copy()
        defaults.update(wheel_diameter=.06,servo_min_us=1100,servo_max_us=1900)
        self.node.feature_config=defaults.copy()
        self.node.enabled=False
        self.values={'kp':.01,'ki':0.,'kd':.002,'max_wz':.08,'track':.13,'yaw_enabled':True,
                     'port':'/dev/ttyAMA3','auto_open':False,**defaults}
        self.node.get_parameter=lambda key:types.SimpleNamespace(value=self.values[key])
        def set_atomic(parameters):
            result=self.node.configure(parameters)
            if result.successful:self.values.update({p.name:p.value for p in parameters})
            return result
        self.node.set_parameters_atomically=set_atomic
        self.gateway=self.gateway_module.Gateway.__new__(self.gateway_module.Gateway)
        self.gateway.node=self.node
        self.gateway.want_arm=False
        self.responses=[]
        self.emitter=patch.object(self.gateway_module,'emit',lambda kind,**kw:self.responses.append(kw))
        self.emitter.start();self.addCleanup(self.emitter.stop)
        parameter_module=types.ModuleType('rclpy.parameter')
        parameter_module.Parameter=lambda name,value:types.SimpleNamespace(name=name,value=value)
        fake_node=types.ModuleType('pi.chassis_node');fake_node.FEATURE_DEFAULTS=defaults
        self.imports=patch.dict(sys.modules,{'rclpy':types.ModuleType('rclpy'),
                                           'rclpy.parameter':parameter_module,'pi.chassis_node':fake_node})
        self.imports.start();self.addCleanup(self.imports.stop)
        self.request={'action':'yaw_set','values':{'kp':.015,'ki':0.,'kd':.003,'max_wz':.08}}

    def test_read_and_apply(self):
        """@brief 读取的是当前节点值，应用成功后控制器和参数值同时更新。"""
        self.gateway.handle({'action':'yaw_get'})
        self.assertEqual(self.responses[-1]['values']['kp'],.01)
        self.gateway.handle(self.request)
        self.assertTrue(self.responses[-1]['success'])
        self.assertFalse(self.responses[-1]['saved'])
        self.assertEqual(self.node.control.kp,.015)
        self.assertEqual(self.values['kp'],.015)

    def test_reject_armed_and_invalid_without_partial_apply(self):
        """@brief 运动中、越界或缺字段请求均不改变任何生效参数。"""
        self.node.enabled=True
        self.gateway.handle(self.request)
        self.assertFalse(self.responses[-1]['success'])
        self.node.enabled=False
        self.gateway.handle({**self.request,'values':{**self.request['values'],'kd':.9}})
        self.assertFalse(self.responses[-1]['success'])
        self.assertEqual(self.values['kp'],.01)
        self.assertEqual(self.node.control.kp,.01)
        self.gateway.handle({**self.request,'values':{'kp':.02}})
        self.assertFalse(self.responses[-1]['success'])

    def test_save_preserves_other_parameters(self):
        """@brief 保存当前全部参数，PID修改不会覆盖轮径、串口或舵机限位。"""
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'chassis_params.yaml'
            with patch.object(self.gateway_module,'save_parameters',
                              side_effect=lambda path,values:save_parameters(target,values)):
                self.gateway.handle({**self.request,'save':True})
            saved=json.loads(target.read_text(encoding='utf-8'))['chassis_controller']['ros__parameters']
            self.assertTrue(self.responses[-1]['saved'])
            self.assertEqual(saved['kp'],.015)
            self.assertEqual(saved['wheel_diameter'],.06)
            self.assertEqual(saved['servo_min_us'],1100)
            self.assertEqual(saved['servo_max_us'],1900)
            self.assertEqual(saved['port'],'/dev/ttyAMA3')

    def test_save_failure_reports_applied_but_not_saved(self):
        """@brief 写盘失败时明确报告已应用但未保存，并返回真实生效值。"""
        with patch.object(self.gateway_module,'save_parameters',side_effect=OSError('disk full')):
            self.gateway.handle({**self.request,'save':True})
        result=self.responses[-1]
        self.assertFalse(result['success'])
        self.assertTrue(result['applied'])
        self.assertFalse(result['saved'])
        self.assertEqual(result['values']['kp'],.015)

    def test_atomic_write_failure_keeps_original(self):
        """@brief 替换失败时旧配置保持完整，并清理临时文件。"""
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'chassis_params.yaml';target.write_text('original',encoding='utf-8')
            with patch('pi.parameter_store.os.replace',side_effect=OSError('denied')):
                with self.assertRaises(OSError):save_parameters(target,self.values)
            self.assertEqual(target.read_text(encoding='utf-8'),'original')
            self.assertEqual(list(Path(directory).iterdir()),[target])


if __name__=='__main__':
    unittest.main()
