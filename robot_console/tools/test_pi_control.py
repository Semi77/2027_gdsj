"""离线验证真实控制代码及串口边界，不访问硬件。"""
import json
import math
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pi.heading_control import HeadingControl
from shared.protocol import WHEEL_COMMAND, FEEDBACK, EXTRA_FEEDBACK, SERVO_COMMAND, Decoder, feedback, extra_feedback, pack_frame
from pi.wheel_odometry import WheelOdometry, quaternion_yaw


class HeadingTests(unittest.TestCase):
    def test_turn_and_reverse(self):
        """@brief 检查左右转映射与前进倒车相同的负反馈方向。"""
        c = HeadingControl()
        self.assertLess(c.update(0., .4, 0., .02)[0], 0.)
        self.assertGreater(c.update(0., .4, 0., .02)[2], 0.)
        self.assertGreater(c.update(0., -.4, 0., .02)[0], 0.)
        for vx in (.1, -.1):
            c.reset()
            c.update(vx, 0., 0., .02)
            wheels = c.update(vx, 0., -5., .02)
            self.assertGreater(wheels[2]-wheels[0], 0.)
            self.assertLessEqual(abs(c.correction), .08)

    def test_closed_loop_disturbance(self):
        """@brief 在简化转向模型中检验前后行驶的误差收敛，模型不代表实车性能。"""
        for vx in (.1, -.1):
            c = HeadingControl()
            c.update(vx, 0., 0., .02)
            yaw = -5.
            for _ in range(1000):
                wheels = c.update(vx, 0., yaw, .02)
                yaw += math.degrees((wheels[2]-wheels[0])/c.track)*.02
            self.assertLess(abs(yaw), .05)

    def test_wrap_and_stop(self):
        """@brief 检查跨界不会形成大角差、松手不会残留转向、换向重新锁定。"""
        c = HeadingControl()
        c.update(.1, 0., 179., .02)
        c.update(.1, 0., -179., .02)
        self.assertLess(c.correction, 0.)
        self.assertEqual(c.update(0., 0., -178., .02), [0.]*4)
        self.assertEqual(c.update(-.1, 0., 40., .02), [-.1]*4)
        with self.assertRaises(ValueError):
            c.update(.1, 0., float('nan'), .02)

    def test_wire_protocol(self):
        """@brief 检查四轮指令的真实字节顺序、CRC、分包和固件版本识别。"""
        body = WHEEL_COMMAND.pack(123, 1, -100, -100, 100, 100)
        self.assertEqual(len(body), 13)
        raw = pack_frame(2, 65535, 42, body)
        decoder = Decoder()
        self.assertEqual(decoder.feed(raw[:7]), [])
        self.assertEqual(decoder.feed(raw[7:]), [(2,65535,42,body)])
        damaged = bytearray(raw); damaged[18] ^= 1
        self.assertEqual(decoder.feed(damaged), [])
        for tag, supported in ((0xC201,False),(0xC202,True)):
            payload = FEEDBACK.pack(tag,2,123,1,*([0]*17),1080)
            self.assertEqual(feedback(payload)['wheel_control'], supported)


def load_node_without_ros():
    """@brief 用最小导入替身加载真实节点方法，测试范围不包含 DDS 或 ROS 运行时。"""
    modules = {}
    for name in ('rclpy','rclpy.node','rclpy.qos','rcl_interfaces','rcl_interfaces.msg',
                 'geometry_msgs','geometry_msgs.msg','std_msgs','std_msgs.msg','std_srvs','std_srvs.srv',
                 'sensor_msgs','sensor_msgs.msg','nav_msgs','nav_msgs.msg'):
        modules[name] = types.ModuleType(name)
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].QoSProfile = object
    modules['rclpy.qos'].DurabilityPolicy = object
    modules['rcl_interfaces.msg'].SetParametersResult = lambda **kw:types.SimpleNamespace(**kw)
    modules['geometry_msgs.msg'].TwistStamped = object
    modules['std_msgs.msg'].String = lambda **kw: types.SimpleNamespace(**kw)
    modules['std_srvs.srv'].SetBool = object
    modules['sensor_msgs.msg'].Imu = object
    def odom_message():
        obj = types.SimpleNamespace
        return obj(header=obj(),pose=obj(pose=obj(position=obj(),orientation=obj()),covariance=[0.]*36),
                   twist=obj(twist=obj(linear=obj(),angular=obj()),covariance=[0.]*36))
    modules['nav_msgs.msg'].Odometry = odom_message
    with patch.dict(sys.modules, modules):
        from pi.chassis_node import ChassisNode
    return ChassisNode


class LeaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 加载真实节点类供协议和租约故障注入使用。"""
        cls.node_type = load_node_without_ros()

    def setUp(self):
        """@brief 构造无硬件节点状态，所有串口写入均进入内存。"""
        self.n = self.node_type.__new__(self.node_type)
        n = self.n
        n.control = HeadingControl(); n.sequence = 0; n.session = 12
        n.feature_config = self.node_type.__init__.__globals__['FEATURE_DEFAULTS'].copy()
        n.odometry = WheelOdometry()
        n.extra = {}
        n.external_yaw = None
        n.external_received = 0.
        n.external_stamp = n.last_servo_stamp = 0
        n.status = {'wheel_control':True,'session':12,'imu_valid':True,'voltage':10.8,
                    'armed':True,'timeout':False,'rpy_deg':[0.,0.,0.]}
        n.enabled = True; n.last_stamp = 0; n.motion = (.1,0.)
        n.arm_pending = False; n.arm_sequence = 0; n.arm_deadline = 0.
        n.deadline = 10.2; n.last_feedback = 10.; n.last_ack_time = 10.
        n.last_tick = n.last_ack = None; n.last_cycle = 10.
        n.decoder = Decoder(); n.on_feedback = None
        self.writes = []
        n.serial = types.SimpleNamespace(write=lambda b:self.writes.append(b) or len(b), read=lambda size:b'')
        n.get_clock = lambda: types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=10_000_000_000))
        self.errors = []
        n.get_logger = lambda:types.SimpleNamespace(error=self.errors.append)

    def test_legacy_cannot_arm(self):
        """@brief 旧固件只能禁能，不能误用新四轮指令使能。"""
        self.n.status['wheel_control'] = False
        with patch('pi.chassis_node.time.monotonic', return_value=10.):
            with self.assertRaises(ValueError): self.n.arm()
        self.n.send(False)
        self.assertEqual(Decoder().feed(self.writes[-1])[0][0],1)

    def test_lost_command_and_ack_stop(self):
        """@brief 模拟命令失联和确认停滞，检查真实 poll 输出禁能帧。"""
        for cause in ('deadline','last_ack_time','last_feedback'):
            self.setUp()
            setattr(self.n,cause,9.)
            with patch('pi.chassis_node.time.monotonic',return_value=10.02): self.n.poll()
            self.assertFalse(self.n.enabled)
            frame = Decoder().feed(self.writes[-1])[0]
            self.assertEqual(WHEEL_COMMAND.unpack(frame[3]),(12,0,0,0,0,0))

    def test_expired_command(self):
        """@brief 已过期 ROS 时间戳不能续租或恢复运动。"""
        vector = types.SimpleNamespace(x=0.,y=0.,z=0.)
        msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=9,nanosec=0)),
                                    twist=types.SimpleNamespace(linear=vector,angular=vector))
        self.n.command(msg)
        self.assertFalse(self.n.enabled)

    def test_valid_feedback_drives_wheel_frame(self):
        """@brief 将真实协议反馈输入节点，确认实际输出四轮帧并等待使能反馈后才运动。"""
        n = self.n
        body = FEEDBACK.pack(0xC202,3,12,1,*([0]*17),1080)
        n.serial.read = lambda size: pack_frame(0x81,1,20,body)
        published = []
        n.feedback_pub = types.SimpleNamespace(publish=published.append)
        with patch('pi.chassis_node.time.monotonic',return_value=10.02): n.poll()
        frame = Decoder().feed(self.writes[-1])[0]
        self.assertEqual(frame[0],2)
        self.assertEqual(WHEEL_COMMAND.unpack(frame[3]),(12,1,100,100,100,100))
        self.assertEqual(json.loads(published[0].data)['firmware'],'wheels-v2')
        n.status['armed'] = False
        n.serial.read = lambda size: b''
        with patch('pi.chassis_node.time.monotonic',return_value=10.04): n.poll()
        self.assertEqual(WHEEL_COMMAND.unpack(Decoder().feed(self.writes[-1])[0][3]),(12,0,0,0,0,0))

    def test_rearm_waits_for_zero_speed_ack(self):
        """@brief 超时后的重新使能等待新的零速确认，不被旧超时反馈误拒绝或提前发送运动。"""
        n=self.n
        n.status.update(armed=False,timeout=True)
        with patch('pi.chassis_node.time.monotonic',return_value=10.):n.arm()
        n.motion=(.1,0.)
        with patch('pi.chassis_node.time.monotonic',return_value=10.02):n.poll()
        self.assertTrue(n.arm_pending)
        self.assertEqual(WHEEL_COMMAND.unpack(Decoder().feed(self.writes[-1])[0][3]),(12,1,0,0,0,0))
        body=FEEDBACK.pack(0xC202,3,12,n.arm_sequence,*([0]*17),1080)
        n.serial.read=lambda size:pack_frame(0x81,1,40,body)
        n.feedback_pub=types.SimpleNamespace(publish=lambda msg:None)
        with patch('pi.chassis_node.time.monotonic',return_value=10.04):n.poll()
        self.assertFalse(n.arm_pending)
        self.assertEqual(WHEEL_COMMAND.unpack(Decoder().feed(self.writes[-1])[0][3]),(12,1,100,100,100,100))

    def test_servo_is_locked_and_independent_of_motor_lease(self):
        """@brief 舵机默认锁定，解锁后独立发送且不延长车轮租约，关闭输出不受解锁限制。"""
        n = self.n
        n.extra = {'session':12,'capabilities':7}
        msg = types.SimpleNamespace(data=json.dumps({'stamp_ns':9_990_000_000,'pulse_us':1500}))
        with patch('pi.chassis_node.time.monotonic',return_value=10.):
            n.servo_command(msg)
            self.assertFalse(self.writes)
            n.feature_config['servo_allowed'] = True
            n.servo_command(msg)
            self.assertEqual(n.deadline,10.2)
            frame = Decoder().feed(self.writes[-1])[0]
            self.assertEqual(frame[0],3)
            self.assertEqual(SERVO_COMMAND.unpack(frame[3]),(12,0,1,1500))
            n.feature_config['servo_allowed'] = False
            msg.data = json.dumps({'stamp_ns':9_995_000_000,'pulse_us':0})
            n.servo_command(msg)
            self.assertEqual(SERVO_COMMAND.unpack(Decoder().feed(self.writes[-1])[0][3]),(12,0,0,0))
            n.disarm()
            self.assertEqual(Decoder().feed(self.writes[-1])[0][0],2)

    def test_external_imu_requires_frame_orientation_and_freshness(self):
        """@brief 外接IMU错误坐标系、无效姿态和过期数据不能用于控制。"""
        n=self.n; obj=types.SimpleNamespace
        n.feature_config['yaw_source']='external'
        msg=obj(header=obj(frame_id='wrong',stamp=obj(sec=9,nanosec=990_000_000)),
                orientation_covariance=[0.]*9,orientation=obj(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        with patch('pi.chassis_node.time.monotonic',return_value=10.):
            n.on_external_imu(msg)
            self.assertIsNone(n.selected_yaw())
            msg.header.frame_id='base_link'
            n.on_external_imu(msg)
            self.assertAlmostEqual(n.selected_yaw(),90.)
        with patch('pi.chassis_node.time.monotonic',return_value=10.3):
            self.assertIsNone(n.selected_yaw())

    def test_mcu_restart_disarms_instead_of_freezing_feedback(self):
        """@brief MCU采样时间回退时禁能并恢复反馈接收，不将重启解释成移动。"""
        n=self.n; n.last_tick=100000
        body=FEEDBACK.pack(0xC202,2,0,0,*([0]*17),1080)
        n.serial.read=lambda size:pack_frame(0x81,1,20,body)
        n.feedback_pub=types.SimpleNamespace(publish=lambda msg:None)
        with patch('pi.chassis_node.time.monotonic',return_value=10.02):n.poll()
        self.assertFalse(n.enabled)
        self.assertEqual(n.last_tick,20)

    def test_extra_feedback_publishes_odometry(self):
        """@brief 实际节点处理扩展反馈后发布里程计，第一次采样仅建立基准。"""
        n=self.n
        outputs=[]
        n.odom_pub=types.SimpleNamespace(publish=outputs.append)
        n.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None))
        data={'session':12,'capabilities':7,'sample_ms':100,'counts':[0]*4}
        with patch('pi.chassis_node.time.monotonic',return_value=10.):
            n.on_extra(data)
            self.assertFalse(outputs)
            n.on_extra({**data,'sample_ms':200,'counts':[104]*4})
        self.assertEqual(len(outputs),1)
        self.assertAlmostEqual(outputs[0].pose.pose.position.x,math.pi*.048/10)
        self.assertEqual(outputs[0].header.frame_id,'odom')

    def test_stale_selected_imu_stops_motors(self):
        """@brief 外接IMU数据过期时实际控制循环禁能，不回退到板载yaw继续运行。"""
        self.n.feature_config['yaw_source']='external'
        self.n.external_yaw=0.
        self.n.external_received=9.
        with patch('pi.chassis_node.time.monotonic',return_value=10.02):self.n.poll()
        self.assertFalse(self.n.enabled)
        self.assertEqual(WHEEL_COMMAND.unpack(Decoder().feed(self.writes[-1])[0][3])[1],0)


class OdometryTests(unittest.TestCase):
    def test_dropped_frames_and_reverse(self):
        """@brief 丢失中间反馈后累计差仍保留总距离，反向计数能退回原位。"""
        o=WheelOdometry()
        self.assertIsNone(o.update([0]*4,0,1,0.))
        result=o.update([104]*4,100,1,0.)
        self.assertAlmostEqual(result['x'],math.pi*.048/10)
        self.assertAlmostEqual(result['vx'],math.pi*.048)
        result=o.update([0]*4,200,1,0.)
        self.assertAlmostEqual(result['x'],0.)

    def test_counter_time_and_yaw_wrap(self):
        """@brief 无符号计数、采样毫秒和yaw同时跨界时仍得到短距离与小转角。"""
        o=WheelOdometry()
        o.update([0xfffffff0]*4,0xfffffff0,1,179.)
        result=o.update([4]*4,24,1,-179.)
        self.assertAlmostEqual(result['dt'],.04)
        self.assertAlmostEqual(result['yaw_rad'],math.radians(2.))
        self.assertLess(result['x'],.004)

    def test_restart_and_duplicate(self):
        """@brief 新会话、重复采样、时间回退和超长缺口均不产生位置跳变。"""
        o=WheelOdometry()
        o.update([100]*4,1000,1,0.)
        self.assertIsNone(o.update([100]*4,1000,1,0.))
        self.assertIsNone(o.update([0]*4,10,1,0.))
        self.assertEqual(o.x,0.)
        self.assertIsNone(o.update([100]*4,20,2,0.))
        self.assertIsNone(o.update([10000]*4,5000,2,0.))
        self.assertEqual(o.x,0.)

    def test_extension_frame_and_quaternion(self):
        """@brief 核查扩展帧布局及无效姿态拒绝行为。"""
        payload=EXTRA_FEEDBACK.pack(20,12,1,2,3,0xffffffff,7,0,1500,9)
        self.assertEqual(len(payload),32)
        data=extra_feedback(payload)
        self.assertEqual(data['counts'],[1,2,3,0xffffffff])
        self.assertEqual(data['servo_ack'],9)
        self.assertAlmostEqual(quaternion_yaw(0,0,0,1),0)
        with self.assertRaises(ValueError):quaternion_yaw(0,0,0,0)


class GatewayTests(unittest.TestCase):
    def test_forward_each_lease_once(self):
        """@brief 相同遥控租约只发布一次，周期发送不能延长旧命令有效期。"""
        modules = {k:types.ModuleType(k) for k in ('fcntl','geometry_msgs','geometry_msgs.msg','rclpy','rclpy.time')}
        def message():
            vector = lambda: types.SimpleNamespace(x=0.,y=0.,z=0.)
            return types.SimpleNamespace(header=types.SimpleNamespace(),
                                         twist=types.SimpleNamespace(linear=vector(),angular=vector()))
        modules['geometry_msgs.msg'].TwistStamped = message
        modules['rclpy.time'].Time = lambda nanoseconds: types.SimpleNamespace(to_msg=lambda:nanoseconds)
        with patch.dict(sys.modules,modules):
            from pi.gateway import Gateway
            g = Gateway.__new__(Gateway)
            g.node = types.SimpleNamespace(serial=object(),disarm=lambda:None,
                get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=10_000_000_000)))
            sent = []
            g.velocity_pub = types.SimpleNamespace(publish=sent.append)
            g.motion_until = 10.15
            g.last_ros_deadline = 0.
            with patch('pi.gateway.time.monotonic',return_value=10.):
                g.send_control(True,.1,0.)
                g.send_control(True,.1,0.)
                self.assertEqual(len(sent),1)
                self.assertAlmostEqual(sent[0].header.stamp/1e9,9.95,places=6)
                g.motion_until = 10.18
                g.send_control(True,.1,0.)
                self.assertEqual(len(sent),2)


if __name__ == '__main__':
    unittest.main()
