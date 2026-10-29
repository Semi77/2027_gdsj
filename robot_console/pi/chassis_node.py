"""车载 ROS2 控制节点：独占串口、yaw 外环、四轮目标及停止保护。"""
import json
import math
from pathlib import Path
import secrets
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shared.protocol import COMMAND, WHEEL_COMMAND, SERVO_COMMAND, Decoder, feedback, extra_feedback, pack_frame
from pi.heading_control import HeadingControl
from pi.wheel_odometry import WheelOdometry, quaternion_yaw

FEATURE_DEFAULTS = {'wheel_diameter': .048, 'counts_per_rev': 1040.,
                    'yaw_source': 'stm32', 'external_imu_frame': 'base_link',
                    'servo_allowed': False, 'servo_min_us': 1000, 'servo_max_us': 2000}


class ChassisNode(Node):
    """每 20ms 执行控制；仅接受新固件的四轮执行接口。"""

    def __init__(self):
        """@brief 建立速度话题、使能服务与参数；默认不打开串口。"""
        super().__init__('chassis_controller')
        self.control = HeadingControl()
        self.odometry = WheelOdometry()
        self.extra = {}
        self.external_yaw = None
        self.external_received = 0.
        self.external_stamp = self.last_servo_stamp = 0
        self.serial = None
        self.decoder = Decoder()
        self.session = secrets.randbits(32) or 1
        self.sequence = 0
        self.enabled = False
        self.arm_pending = False
        self.arm_sequence = 0
        self.arm_deadline = 0.
        self.status = {}
        self.last_feedback = self.last_ack_time = self.deadline = 0.
        self.last_tick = self.last_ack = None
        self.last_stamp = 0
        self.last_cycle = time.monotonic()
        self.motion = (0., 0.)
        self.on_feedback = None
        self.last_error = ''
        for name, value in {'kp': .010, 'ki': .0, 'kd': .002, 'max_wz': .08,
                            'track': .130, 'yaw_enabled': True, 'port': '/dev/ttyAMA3',
                            'auto_open': False, **FEATURE_DEFAULTS}.items():
            self.declare_parameter(name, value)
        self.apply_parameters()
        self.add_on_set_parameters_callback(self.configure)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self.command_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.command, qos)
        self.feedback_pub = self.create_publisher(String, '/chassis/feedback', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        from rclpy.qos import qos_profile_sensor_data
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.on_external_imu, qos_profile_sensor_data)
        self.servo_sub = self.create_subscription(String, '/servo/command', self.servo_command, qos)
        self.arm_service = self.create_service(SetBool, '/chassis/enable', self.enable_service)
        self.timer = self.create_timer(.02, self.poll)
        if self.get_parameter('auto_open').value:
            self.open_port(self.get_parameter('port').value)

    def apply_parameters(self):
        """@brief 将启动参数应用到航向控制器。"""
        values = {key: self.get_parameter(key).value for key in
                  ('kp', 'ki', 'kd', 'max_wz', 'track', 'yaw_enabled')}
        self.validate_parameters(values)
        for key, value in values.items():
            setattr(self.control, 'enabled' if key == 'yaw_enabled' else key, value)
        self.feature_config = {key:self.get_parameter(key).value for key in FEATURE_DEFAULTS}
        self.validate_parameters(self.feature_config)
        self.apply_features()

    def apply_features(self):
        """@brief 更新树莓派换算参数并重建里程计采样基准。"""
        self.odometry.diameter = self.feature_config['wheel_diameter']
        self.odometry.counts_per_rev = self.feature_config['counts_per_rev']
        self.odometry.reset_reference()
        self.external_yaw = None
        self.external_received = 0.
        self.external_stamp = 0

    @staticmethod
    def validate_parameters(values):
        """@brief 校验参数范围，values 为待修改参数字典。"""
        limits = {'kp': (0., .2), 'ki': (0., .02), 'kd': (0., .05),
                  'max_wz': (0., .3), 'track': (.05, .5), 'wheel_diameter':(.01,.5),
                  'counts_per_rev':(1.,1000000.), 'servo_min_us':(500,2500), 'servo_max_us':(500,2500)}
        for key, value in values.items():
            if key in limits:
                lo, hi = limits[key]
                if isinstance(value, bool) or not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(f'{key} 超出范围 {lo}..{hi}')
            elif key in ('yaw_enabled','servo_allowed') and not isinstance(value, bool):
                raise ValueError(f'{key} 必须为布尔值')
            elif key == 'yaw_source' and value not in ('stm32','external'):
                raise ValueError('yaw_source 必须为 stm32 或 external')
            elif key == 'external_imu_frame' and (not isinstance(value,str) or not value):
                raise ValueError('external_imu_frame 不能为空')
            if key in ('servo_min_us','servo_max_us') and type(value) is not int:
                raise ValueError('舵机脉宽限位必须为整数')
        if values.get('servo_min_us',500) > values.get('servo_max_us',2500):
            raise ValueError('舵机最小脉宽不能大于最大脉宽')

    def configure(self, parameters):
        """@brief 禁能时允许在线调节航向参数，运行中拒绝修改。"""
        try:
            values = {p.name: p.value for p in parameters}
            if self.enabled or any(k not in ('kp','ki','kd','max_wz','track','yaw_enabled',*FEATURE_DEFAULTS) for k in values):
                raise ValueError('请先禁能；串口配置仅在启动时修改')
            self.validate_parameters({**self.feature_config, **values})
            for key, value in values.items():
                if key in FEATURE_DEFAULTS:
                    self.feature_config[key] = value
                else:
                    setattr(self.control, 'enabled' if key == 'yaw_enabled' else key, value)
            self.apply_features()
            self.control.reset()
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def open_port(self, port):
        """@brief 独占指定底盘串口，禁止访问雷达与调试串口。"""
        resolved = str(Path(port).resolve())
        if not (resolved == '/dev/ttyAMA3' or resolved.startswith(('/dev/ttyUSB','/dev/ttyACM'))):
            raise ValueError('请选择底盘 UART3 或 USB 串口')
        self.close_port()
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=.05, exclusive=True)
        self.serial.reset_input_buffer()
        self.decoder = Decoder()
        self.status = {}
        self.last_feedback = 0.
        self.last_tick = self.last_ack = None
        self.extra = {}
        self.odometry.reset_reference()
        self.session = secrets.randbits(32) or 1
        self.disarm()

    def send(self, enabled, wheels=None):
        """@brief 仅此入口写串口；旧固件仅发送兼容禁能帧。"""
        if self.serial is None:
            return
        self.sequence = (self.sequence+1) & 65535
        if self.status.get('wheel_control'):
            payload = WHEEL_COMMAND.pack(self.session, int(enabled),
                                        *(round(v*1000) for v in (wheels or [0.]*4)))
            kind = 2
        else:
            if enabled:
                raise ValueError('需要先烧入 wheels-v2 固件，旧固件仅允许监测和禁能')
            payload, kind = COMMAND.pack(self.session, 0, 0, 0), 1
        packet = pack_frame(kind, self.sequence, int(time.monotonic()*1000), payload)
        if self.serial.write(packet) != len(packet):
            raise OSError('底盘串口短写')

    def disarm(self):
        """@brief 清除速度租约与航向锁定，并发送禁能指令。"""
        self.enabled = False
        self.arm_pending = False
        self.deadline = 0.
        self.motion = (0., 0.)
        self.control.reset()
        self.send(False)

    def arm(self):
        """@brief 新鲜反馈、会话同步、IMU 和电压正常后零速使能。"""
        now = time.monotonic()
        if not self.serial or now-self.last_feedback > .2:
            raise ValueError('底盘反馈超时')
        if not self.status.get('wheel_control'):
            raise ValueError('需烧入 wheels-v2 固件，当前仅允许监测')
        if self.status.get('session') != self.session or not self.status.get('imu_valid'):
            raise ValueError('底盘会话或 IMU 未就绪')
        if self.status['voltage'] < 10.:
            raise ValueError('底盘电压低于 10V')
        if self.selected_yaw() is None:
            raise ValueError('所选航向来源无有效新鲜数据')
        self.control.reset()
        self.motion = (0., 0.)
        self.last_stamp = 0
        self.enabled = True
        self.arm_pending = True
        self.arm_deadline = now+.2
        self.deadline = now+.2
        self.last_ack_time = now
        self.send(True)
        self.arm_sequence = self.sequence

    def enable_service(self, request, response):
        """@brief SetBool 服务显式使能或禁能，重连后不恢复旧指令。"""
        try:
            self.arm() if request.data else self.disarm()
            response.success, response.message = True, '已提交零速使能' if request.data else '已禁能'
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def command(self, msg):
        """@brief 接收带时间戳的车体速度；过期、重复或超限指令导致禁能。"""
        if not self.enabled:
            return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        vx, wz = msg.twist.linear.x, msg.twist.angular.z
        others = (msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y)
        if (stamp <= self.last_stamp or not 0 <= age < .2 or
            not all(math.isfinite(v) for v in (vx,wz,*others)) or
            abs(vx) > .2 or abs(wz) > .7 or any(v != 0. for v in others)):
            self.disarm()
            return
        self.last_stamp = stamp
        self.motion = (vx,wz)
        self.deadline = time.monotonic()+(.2-age)

    def poll(self):
        """@brief 50Hz 收反馈、检查租约及确认进展、计算航向和发送四轮目标。"""
        now = time.monotonic()
        dt, self.last_cycle = now-self.last_cycle, now
        if not self.serial:
            return
        try:
            for kind, seq, tick, payload in self.decoder.feed(self.serial.read(4096)):
                if kind == 0x82:
                    self.on_extra(extra_feedback(payload))
                    continue
                if kind != 0x81:
                    continue
                data = feedback(payload)
                if self.last_tick is not None:
                    delta = (tick-self.last_tick)&0xffffffff
                    if delta == 0:
                        continue
                    if delta >= 0x80000000:
                        self.disarm()
                        self.odometry.reset_reference()
                        self.extra = {}
                self.last_tick = tick
                self.last_feedback, self.status = now, data
                if data['session'] == self.session and data['ack'] != self.last_ack:
                    self.last_ack, self.last_ack_time = data['ack'], now
                if (self.arm_pending and data['session'] == self.session and data['armed'] and
                    ((data['ack']-self.arm_sequence)&65535) < 32768):
                    self.arm_pending = False
                message = {'tick': tick, **data, 'yaw_correction': self.control.correction,
                           'yaw_source':self.feature_config['yaw_source'],
                           'servo_allowed':self.feature_config['servo_allowed'],
                           'servo_min_us':self.feature_config['servo_min_us'],
                           'servo_max_us':self.feature_config['servo_max_us'],
                           'extra':self.extra}
                self.feedback_pub.publish(String(data=json.dumps(message)))
                if self.on_feedback:
                    self.on_feedback(message)
            if self.enabled and (now >= self.deadline or now-self.last_feedback > .2 or
                now-self.last_ack_time > .2 or not self.status.get('imu_valid') or
                self.status.get('session') != self.session or not self.status.get('wheel_control') or
                ((self.status.get('timeout') or not self.status.get('armed')) and not self.arm_pending) or
                (self.arm_pending and now >= self.arm_deadline) or self.status.get('voltage',0) < 10. or
                self.selected_yaw() is None or not 0 < dt <= .2):
                self.disarm()
            wheels = [0.]*4
            if self.enabled and not self.arm_pending and self.status.get('armed'):
                wheels = self.control.update(*self.motion, self.selected_yaw(), dt)
            self.send(self.enabled, wheels)
        except (OSError, ValueError) as error:
            self.last_error = str(error)
            self.get_logger().error(self.last_error)
            self.close_port()

    def on_external_imu(self, msg):
        """@brief 接收已转换到指定车体坐标系的姿态，拒绝无姿态、旧时间戳和无效四元数。"""
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (msg.header.frame_id != self.feature_config['external_imu_frame'] or
            not math.isfinite(msg.orientation_covariance[0]) or msg.orientation_covariance[0] < 0 or
            stamp <= self.external_stamp or not 0 <= age < .2):
            return
        try:
            q = msg.orientation
            self.external_yaw = quaternion_yaw(q.x,q.y,q.z,q.w)
            self.external_received = time.monotonic()-age
            self.external_stamp = stamp
        except ValueError:
            self.external_yaw = None

    def selected_yaw(self):
        """@brief 返回新鲜有效的所选航向；外接IMU失效时不静默切换到另一来源。"""
        if self.feature_config['yaw_source'] == 'external':
            return self.external_yaw if time.monotonic()-self.external_received < .2 else None
        if self.status.get('imu_valid') and time.monotonic()-self.last_feedback < .2:
            return self.status['rpy_deg'][2]
        return None

    def on_extra(self, data):
        """@brief 用MCU采样时刻和累计计数生成局部里程计，不把反馈接收时刻当采样间隔。"""
        self.extra = data
        yaw = self.selected_yaw()
        if data['session'] != self.session or not data['capabilities'] & 1 or yaw is None:
            self.odometry.reset_reference()
            return
        result = self.odometry.update(data['counts'], data['sample_ms'], data['session'], yaw)
        if result is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = result['x'], result['y']
        msg.pose.pose.orientation.z = math.sin(result['yaw_rad']/2)
        msg.pose.pose.orientation.w = math.cos(result['yaw_rad']/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = result['vx'], result['wz']
        # 保守初值，尚未经实测标定；未测的高度、横滚和俯仰使用高方差。
        for i,v in enumerate((.04,.04,1e6,1e6,1e6,.04)):
            msg.pose.covariance[i*7] = v
        for i,v in enumerate((.04,1.,1e6,1e6,1e6,.04)):
            msg.twist.covariance[i*7] = v
        self.odom_pub.publish(msg)

    def servo_command(self, msg):
        """@brief 接收带时间戳的单次舵机脉宽JSON，失联不覆盖最后位置。"""
        try:
            data = json.loads(msg.data)
            stamp = data['stamp_ns']
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            pulse = data['pulse_us']
            if type(stamp) is not int or stamp <= self.last_servo_stamp or not 0 <= age < .2:
                raise ValueError('舵机请求过期或重复')
            if type(pulse) is not int:
                raise ValueError('舵机脉宽必须为整数')
            if (not self.serial or time.monotonic()-self.last_feedback > .2 or
                self.extra.get('session') != self.session or not self.extra.get('capabilities',0) & 2):
                raise ValueError('舵机执行接口未就绪')
            if pulse != 0 and (not self.feature_config['servo_allowed'] or not
                self.feature_config['servo_min_us'] <= pulse <= self.feature_config['servo_max_us']):
                raise ValueError('舵机未解锁或脉宽超出已配置限位')
            self.last_servo_stamp = stamp
            self.sequence = (self.sequence+1)&65535
            packet = pack_frame(3,self.sequence,int(time.monotonic()*1000),
                                SERVO_COMMAND.pack(self.session,0,int(pulse!=0),pulse))
            if self.serial.write(packet) != len(packet):
                raise OSError('舵机指令短写')
        except (ValueError, TypeError, KeyError, OSError) as error:
            self.get_logger().error(str(error))

    def close_port(self):
        """@brief 尽力禁能并释放串口，写失败时依靠 MCU 的独立超时停车。"""
        try:
            self.disarm()
        except OSError:
            pass
        finally:
            if self.serial:
                self.serial.close()
            self.serial = None


def main():
    """@brief 独立运行 ROS 控制节点，退出时释放底盘。"""
    rclpy.init()
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_port()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
