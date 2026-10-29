"""通过 SSH 标准输入输出工作的 ROS 2 网关，断开时清理本会话资源。"""
import fcntl
import glob
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shared.protocol import finite_number
from shared.results import validate_detection
from pi.parameter_store import save_parameters
from shared.navigation_geometry import MapGeometry,pose_values

YAW_PARAMETERS = ('kp', 'ki', 'kd', 'max_wz')

OUT_LOCK = threading.Lock()
OUTPUT_ALIVE = True


def emit(kind, **values):
    """@brief 串行输出一条 JSON 消息，拒绝 NaN 避免污染数据接口。"""
    global OUTPUT_ALIVE
    with OUT_LOCK:
        if not OUTPUT_ALIVE:
            return
        try:
            print(json.dumps({'type': kind, **values}, ensure_ascii=False,
                             allow_nan=False, separators=(',', ':')), flush=True)
        except BrokenPipeError:
            OUTPUT_ALIVE = False


class Gateway:
    def __init__(self):
        """@brief 初始化通信会话，默认禁能且不自动打开底盘串口。"""
        self.commands = queue.Queue(maxsize=100)
        self.running = True
        self.session = 0
        self.last_feedback = 0.
        self.last_status = {}
        self.motion_until = 0.
        self.motion = (0., 0.)
        self.want_arm = False
        self.processes = {}
        self.logs = Path('/workspace/ros2_ws/robot_console/logs')
        self.logs.mkdir(exist_ok=True)
        self.node = None
        self.navigation = None
        self.scene = None
        self.current_map = None
        self.last_scan_message = None
        self.subscriptions = []
        self.last_publish = {}
        self.last_system = 0.
        self.last_send = 0.
        self.last_state = 0.
        self.last_ros_deadline = 0.
        self.setup_ros()

    def setup_ros(self):
        """@brief 建立车载控制节点与标准 ROS 订阅，缺少 ROS 时只提供诊断。"""
        try:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
            from sensor_msgs.msg import Imu, LaserScan
            from nav_msgs.msg import OccupancyGrid
            from std_msgs.msg import String
            config = Path(__file__).with_name('chassis_params.yaml')
            rclpy.init(args=['--ros-args', '--params-file', str(config)] if config.exists() else [])
            from pi.chassis_node import ChassisNode
            from geometry_msgs.msg import TwistStamped
            self.node = ChassisNode()
            self.node.on_feedback = self.on_chassis_feedback
            self.session = self.node.session
            self.velocity_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 1)
            self.servo_pub = self.node.create_publisher(String, '/servo/command', 1)
            try:
                from pi.navigation_control import NavigationControl
                self.navigation = NavigationControl(self, emit)
                from pi.navigation_scene import NavigationScene
                self.scene = NavigationScene(self)
            except ImportError as error:
                emit('log', text=f'导航组件尚未安装：{error}')
            self.rclpy = rclpy
            for topic in ['/imu/data', '/imu/data_raw']:
                self.subscriptions.append(self.node.create_subscription(
                    Imu, topic, lambda msg, t=topic: self.on_imu(t, msg), qos_profile_sensor_data))
            self.subscriptions.append(self.node.create_subscription(
                LaserScan, '/scan', self.on_scan, qos_profile_sensor_data))
            self.subscriptions.append(self.node.create_subscription(
                OccupancyGrid, '/map', self.on_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            for topic, source in [('/k230/detections_json', 'k230'), ('/pi/results_json', 'pi')]:
                self.subscriptions.append(self.node.create_subscription(
                    String, topic, lambda msg, s=source: self.on_result(s, msg.data), 10))
        except ImportError as error:
            emit('log', text=f'ROS 订阅不可用：{error}')

    def due(self, key, period):
        """@brief 按单调时间限制界面传输频率，不改变 ROS 原始采样率。"""
        now = time.monotonic()
        if now - self.last_publish.get(key, 0) < period:
            return False
        self.last_publish[key] = now
        return True

    def on_imu(self, topic, msg):
        """@brief 输出原始惯性量；仅在姿态协方差有效时计算欧拉角。"""
        if not self.due(topic, .05):
            return
        data = {'source': topic, 'gyro': [msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z], 'accel': [msg.linear_acceleration.x,
                msg.linear_acceleration.y, msg.linear_acceleration.z], 'valid': False}
        q = msg.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if msg.orientation_covariance[0] >= 0 and .9 < norm < 1.1:
            x, y, z, w = (v / math.sqrt(norm) for v in (q.x, q.y, q.z, q.w))
            data['rpy_deg'] = [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
                math.degrees(math.asin(max(-1., min(1., 2*(w*y-z*x))))),
                math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]
            data['valid'] = True
        if all(math.isfinite(v) for v in data['gyro'] + data['accel']):
            emit('imu', **data)

    def on_scan(self, msg):
        """@brief 将激光转换为自身坐标系的二维点，禁止冒充地图坐标。"""
        self.last_scan_message=msg
        if self.navigation:
            self.navigation.last_scan = time.monotonic()
        if not self.due('scan', .15):
            return
        points = []
        for i in range(0, len(msg.ranges), max(1, len(msg.ranges)//720)):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                points.append([round(r*math.cos(a), 3), round(r*math.sin(a), 3)])
        emit('scan', frame=msg.header.frame_id, points=points)

    def on_map(self, msg):
        """@brief 传输有效二维地图，限制尺寸以控制界面内存占用。"""
        if msg.info.width<=0 or msg.info.height<=0 or msg.info.width*msg.info.height>1000000:
            return
        q=msg.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.current_map={'frame':msg.header.frame_id, 'width':msg.info.width,'height':msg.info.height,
            'resolution':msg.info.resolution, 'origin':[msg.info.origin.position.x,msg.info.origin.position.y,yaw],
            'data':list(msg.data)}
        if self.due('map',1.):emit('map',**self.current_map)

    def map_pose_command(self, command):
        """@brief 校验Windows位置命令的期限及空闲栅格，再调用ROS定位或导航接口。"""
        action=command['action']
        try:
            deadline=finite_number(command.get('deadline',0),1e12)
            if not 0<deadline-time.time()<=1.1:raise ValueError('地图选点已过期，请重新选择')
            if not self.navigation or not self.current_map:raise ValueError('地图或导航组件尚未就绪')
            x,y,yaw=pose_values(command)
            geometry=MapGeometry(self.current_map)
            if geometry.frame!='map' or not geometry.free(self.current_map['data'],x,y):
                raise ValueError('选点必须在map地图的已知空闲区域')
            values={'x':x,'y':y,'yaw':yaw}
            if action=='navigation_initial_pose':self.navigation.set_initial_pose(values)
            else:self.navigation.submit_goal(values)
            emit('map_command_result',action=action,success=True,text=self.navigation.result)
        except (ValueError,TypeError,KeyError) as error:
            emit('map_command_result',action=action,success=False,text=str(error))

    def on_result(self, source, raw):
        """@brief 接收树莓派或 K230 JSON 结果，按来源独立展示。"""
        if len(raw) > 65536 or not self.due(source, .1):
            return
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError('顶层必须为对象')
            if source == 'k230':
                validate_detection(data)
            emit('result', source=source, data=data)
        except (ValueError, TypeError) as error:
            emit('log', text=f'{source} 结果格式错误：{error}')

    def input_loop(self):
        """@brief 有界读取 SSH 输入，EOF 或输入拥塞立即结束控制会话。"""
        try:
            while self.running:
                line = sys.stdin.buffer.readline(65537)
                if not line or len(line) > 65536:
                    break
                try:
                    command = json.loads(line)
                    if not isinstance(command, dict):
                        raise ValueError('请求必须为对象')
                    self.commands.put_nowait(command)
                except (ValueError, queue.Full):
                    break
        finally:
            self.running = False

    @property
    def serial(self):
        """@brief 查询 ROS 控制节点是否已打开串口，网关自身不写串口。"""
        return self.node.serial if self.node else None

    def on_chassis_feedback(self, data):
        """@brief 将 ROS 控制节点采集的底盘反馈送往桌面。"""
        self.last_feedback = time.monotonic()
        self.last_status = data
        emit('wheels', **data)

    def send_control(self, enabled, vx=0., wz=0.):
        """@brief 将剩余遥控租约转换为 ROS 时间戳，禁止网关重复发送延长租约。"""
        if not self.node or not self.serial:
            return
        if not enabled:
            self.node.disarm()
            self.last_ros_deadline = 0.
            return
        from geometry_msgs.msg import TwistStamped
        from rclpy.time import Time
        remaining = min(.2, self.motion_until-time.monotonic())
        if remaining <= 0:
            self.node.disarm()
            return
        if self.motion_until <= self.last_ros_deadline:
            return
        msg = TwistStamped()
        msg.header.stamp = Time(nanoseconds=self.node.get_clock().now().nanoseconds-
                               int((.2-remaining)*1e9)).to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        self.velocity_pub.publish(msg)
        self.last_ros_deadline = self.motion_until

    def disarm(self):
        """@brief 撤销遥控租约并发送禁能帧，重连后不会恢复旧速度。"""
        if self.navigation and self.navigation.active:
            self.navigation.stop()
        self.want_arm = False
        self.motion_until = 0.
        self.motion = (0., 0.)
        self.send_control(False)

    def open_serial(self, port):
        """@brief 由 ROS 控制节点独占打开底盘接口。"""
        if not self.node:
            raise ValueError('ROS2 控制节点不可用')
        self.node.open_port(port)
        self.session = self.node.session
        self.last_feedback = 0.
        self.last_status = {}
        self.disarm()
        emit('log', text=f'已打开 {port}，运动需要 wheels-v2 固件')

    def yaw_parameters(self, command):
        """@brief 读取或原子应用四个yaw参数，保存时写入节点当前全部配置。"""
        applied = False
        try:
            if self.node is None:
                raise ValueError('树莓派ROS控制节点不可用')
            if command['action'] == 'yaw_set':
                if self.want_arm or self.node.enabled:
                    raise ValueError('请先禁能小车，再应用参数')
                values = command.get('values')
                if not isinstance(values,dict) or set(values) != set(YAW_PARAMETERS):
                    raise ValueError('请完整提供Kp、Ki、Kd和纠偏限幅')
                if any(type(values[key]) not in (int,float) for key in YAW_PARAMETERS):
                    raise ValueError('参数必须为数值')
                from rclpy.parameter import Parameter
                result = self.node.set_parameters_atomically([
                    Parameter(key, value=finite_number(values[key],1.)) for key in YAW_PARAMETERS])
                if not result.successful:
                    raise ValueError(result.reason)
                applied = True
                if command.get('save',False):
                    from pi.chassis_node import FEATURE_DEFAULTS
                    keys = (*YAW_PARAMETERS,'track','yaw_enabled','port','auto_open',*FEATURE_DEFAULTS)
                    all_values = {key:self.node.get_parameter(key).value for key in keys}
                    save_parameters(Path(__file__).with_name('chassis_params.yaml'),all_values)
            values = {key:self.node.get_parameter(key).value for key in YAW_PARAMETERS}
            emit('yaw_parameters', success=True, values=values, saved=bool(applied and command.get('save',False)),
                 applied=applied, text='已应用并保存，重启后仍生效' if applied and command.get('save',False) else
                 '已应用到本次运行，尚未保存' if applied else '已读取树莓派当前生效参数')
        except (ValueError,TypeError,OSError) as error:
            emit('yaw_parameters', success=False, applied=applied, saved=False,
                 values={key:self.node.get_parameter(key).value for key in YAW_PARAMETERS} if applied else None,
                 text=('参数已应用，但保存失败：' if applied else '参数操作失败：')+str(error))

    def handle(self, command):
        """@brief 分派固定操作，不接受上位机发来的任意 shell 命令。"""
        action = command.get('action')
        if action in ('yaw_get','yaw_set'):
            self.yaw_parameters(command)
        elif action == 'ports':
            ports = sorted(glob.glob('/dev/serial/by-id/*'))
            if not ports:
                ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
            # GPIO UART3 对接 PC10/PC11，优先列出且不枚举雷达 UART0。
            if Path('/dev/ttyAMA3').exists():
                ports.insert(0, '/dev/ttyAMA3')
            emit('ports', ports=ports)
        elif action == 'serial_open':
            self.open_serial(str(command['port']))
        elif action == 'serial_close':
            self.disarm()
            if self.node:
                self.node.close_port()
            self.last_feedback = 0.
            self.last_status = {}
        elif action == 'disarm':
            self.disarm()
        elif action == 'servo':
            deadline = finite_number(command.get('deadline',0),1e12)
            remaining = deadline-time.time()
            pulse = command.get('pulse_us')
            if not self.node or not 0 < remaining <= .2 or type(pulse) is not int:
                raise ValueError('舵机命令无效或已过期')
            from std_msgs.msg import String
            stamp = self.node.get_clock().now().nanoseconds-int((.2-remaining)*1e9)
            self.servo_pub.publish(String(data=json.dumps({'stamp_ns':stamp,'pulse_us':pulse})))
        elif action in ('arm', 'motion'):
            if self.navigation and self.navigation.active:
                raise ValueError('自主控制期间不能遥控，请先取消自主控制')
            deadline = finite_number(command.get('deadline', 0), 1e12)
            if not 0 < deadline - time.time() <= .5:
                self.disarm()
                raise ValueError('遥控消息已过期，请检查连接延迟和系统时间')
            if time.monotonic() - self.last_feedback > .2:
                self.disarm()
                raise ValueError('底盘反馈不新鲜，拒绝使能')
            if action == 'arm':
                if self.last_status.get('session') != self.session:
                    raise ValueError('底盘会话未同步')
                if not self.last_status.get('imu_valid'):
                    raise ValueError('固件惯性初始化或静止校准尚未完成')
                self.node.arm()
                self.want_arm = True
                self.motion = (0., 0.)
                self.motion_until = time.monotonic() + .15
                self.send_control(True)
                emit('control', armed=True)
            else:
                if not self.want_arm:
                    return
                self.motion = (finite_number(command['vx'], .2), finite_number(command['wz'], .7))
                self.motion_until = time.monotonic() + min(.2, deadline-time.time())
        elif action == 'map_refresh':
            if self.current_map:emit('map',**self.current_map)
            else:emit('log',text='尚未收到地图，请先启动建图或地图定位导航')
        elif action in ('navigation_initial_pose','navigation_goal'):
            self.map_pose_command(command)
        elif action == 'navigation_enable':
            deadline = finite_number(command.get('deadline',0),1e12)
            if not 0 < deadline-time.time() <= .5:
                raise ValueError('自主控制授权已过期，请重新点击')
            if not self.navigation:
                raise ValueError('导航组件尚未安装')
            self.navigation.enable()
            emit('navigation', **self.navigation.status())
        elif action == 'navigation_cancel':
            if self.navigation:
                self.navigation.stop()
            self.disarm()
            emit('control', armed=False)
        elif action == 'start':
            self.disarm()
            emit('control', armed=False)
            self.start_program(command['name'])
        elif action == 'stop':
            self.disarm()
            emit('control', armed=False)
            self.stop_program(command['name'])
        elif action == 'save_map':
            self.disarm()
            emit('control', armed=False)
            self.start_program('save_map')
        elif action == 'shutdown':
            self.running = False
        else:
            raise ValueError('未知操作')

    def start_program(self, name):
        """@brief 按白名单启动本会话程序，并为每个程序创建独立进程组。"""
        base = '/workspace/ros2_ws'
        programs = {
            'lidar': ['ros2', 'run', 'delta2c_lidar', 'lidar_node'],
            'mapping': ['ros2', 'launch', f'{base}/lidar_support/mapping/mapping.launch.py', 'rviz:=false'],
            'rviz': ['rviz2', '-d', f'{base}/lidar_support/mapping/mapping.rviz'],
            'navigation': ['ros2', 'launch', str(Path(__file__).with_name('navigation.launch.py'))],
            'navigation_rviz': ['rviz2', '-d', str(Path(__file__).with_name('navigation.rviz'))],
            'save_map': ['python3', f'{base}/lidar_support/mapping/save_map.py',
                         f'{base}/maps/console_map'],
        }
        if name not in programs:
            raise ValueError('该预设程序尚未接入')
        if name in self.processes and self.processes[name][0].poll() is None:
            raise ValueError('本会话中程序已经运行')
        if self.node:
            nodes = self.node.get_node_names()
            if name == 'navigation':
                if 'cartographer_node' in nodes:
                    raise ValueError('请先停止纯雷达建图，再启动地图定位导航')
                if any(n in nodes for n in ('amcl','controller_server','navigation_tf')):
                    raise ValueError('已有导航节点运行，请先停止原实例')
                if not self.serial:
                    raise ValueError('请先打开底盘串口，以提供真实里程计')
                if not Path(f'{base}/maps/console_map.yaml').is_file():
                    raise ValueError('未找到console_map.yaml，请先保存地图')
                if self.navigation:
                    self.navigation.pose_valid = False
            if name == 'mapping' and any(n in nodes for n in ('amcl','navigation_tf')):
                raise ValueError('请先停止定位导航和导航RViz，再启动纯雷达建图')
            # 仅显示雷达时使用 laser 坐标系，建图已运行时沿用 map 配置。
            if name == 'rviz' and 'cartographer_node' not in nodes:
                programs['rviz'] = ['rviz2', '-d', str(Path(__file__).with_name('lidar.rviz'))]
            if name == 'lidar' and 'delta2c_lidar' in nodes:
                raise ValueError('检测到外部雷达节点，请先在原终端停止，避免重复占用串口')
            if name == 'mapping' and 'cartographer_node' in nodes:
                raise ValueError('检测到外部建图节点，请先停止旧实例')
        log = open(self.logs / f'{name}.log', 'a', encoding='utf-8')
        environment = os.environ.copy()
        if name in ('rviz','navigation_rviz'):
            auth = f'{base}/robot_console/.Xauthority'
            if not Path(auth).exists():
                log.close()
                raise ValueError('未找到树莓派桌面的 Xauthority，请先登录桌面后重新连接上位机')
            environment.update(DISPLAY=':0', XAUTHORITY=auth, QT_QPA_PLATFORM='xcb',
                               QT_X11_NO_MITSHM='1', LIBGL_ALWAYS_SOFTWARE='1')
        try:
            process = subprocess.Popen(programs[name], cwd=base, stdout=log, stderr=log, env=environment,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError:
            log.close()
            raise
        self.processes[name] = (process, log, 0)
        emit('log', text=f'已启动 {name}，PID {process.pid}；数据是否就绪以实时页面为准')

    def stop_program(self, name):
        """@brief 按进程组停止网关拥有的程序，不能停止外部会话程序。"""
        if name == 'navigation' and self.navigation:
            self.navigation.stop('导航程序已停止')
        if name not in self.processes:
            return
        process, log, _ = self.processes[name]
        # 即使父进程已退出，也清理仍在同组的 ROS 子进程。
        for sig, wait in [(signal.SIGINT, 3), (signal.SIGTERM, 1), (signal.SIGKILL, 1)]:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            deadline=time.monotonic()+wait
            while time.monotonic()<deadline:
                process.poll()
                try:
                    os.killpg(process.pid,0)
                except ProcessLookupError:
                    break
                time.sleep(.05)
        log.close()
        del self.processes[name]
        emit('log', text=f'已停止 {name}')

    def poll(self):
        """@brief 非阻塞处理串口和遥控期限，过期后需重新人工使能。"""
        now = time.monotonic()
        navigation_active = self.navigation and self.navigation.active
        if self.navigation:
            self.navigation.tick()
        if self.scene and self.due('navigation_scene',.2):
            emit('navigation_scene',**self.scene.snapshot())
        if self.serial:
            try:
                if not navigation_active and self.want_arm and (now >= self.motion_until or now-self.last_feedback > .2
                                      or not self.node.enabled):
                    self.disarm()
                    emit('log', text='遥控租约、反馈或控制状态失效，已禁能，需重新使能')
                if not navigation_active and now-self.last_send >= .05:
                    self.last_send = now
                    self.send_control(self.want_arm, *self.motion)
            except (OSError, ValueError) as error:
                self.want_arm = False
                self.node.close_port()
                self.last_feedback = 0.
                emit('log', text=f'底盘控制异常：{error}')
        else:
            self.want_arm = False
        if now-self.last_system >= 1.:
            self.last_system = now
            try:
                temperature = int(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000
            except (OSError, ValueError):
                temperature = None
            emit('system', hostname=os.uname().nodename, load=os.getloadavg()[0],
                 temperature=temperature, ros=self.node is not None)
        if now-self.last_state >= .5:
            self.last_state = now
            if self.navigation:
                emit('navigation', **self.navigation.status())
            emit('state', armed=bool(self.node and self.node.enabled), serial=bool(self.serial),
                 feedback_age=round(now-self.last_feedback, 2) if self.last_feedback else None,
                 programs={name: proc.poll() for name, (proc, _, _) in self.processes.items()})
            for name, (proc, log, offset) in list(self.processes.items()):
                path = self.logs / f'{name}.log'
                if path.stat().st_size > 5_000_000:
                    # 子进程日志以追加方式写入，可安全截断并重新读取后续内容。
                    with open(path,'w',encoding='utf-8'):
                        pass
                    offset=0
                with open(path, encoding='utf-8', errors='replace') as reader:
                    reader.seek(offset)
                    tail = reader.read(8192)
                    self.processes[name] = (proc, log, reader.tell())
                if tail:
                    emit('log', text=f'[{name}] {tail}')

    def run(self):
        """@brief 执行会话循环，任意退出路径都禁能并回收本会话程序。"""
        threading.Thread(target=self.input_loop, daemon=True).start()
        emit('hello', version=1, server_time=time.time(), session=self.session)
        try:
            while self.running and OUTPUT_ALIVE:
                for _ in range(20):
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self.handle(command)
                    except Exception as error:
                        emit('log', text=f'操作失败：{error}')
                self.poll()
                if self.node:
                    self.rclpy.spin_once(self.node, timeout_sec=.005)
                else:
                    time.sleep(.005)
        finally:
            try:
                self.disarm()
            except OSError:
                self.want_arm=False
            if self.node:
                self.node.close_port()
            for name in list(self.processes):
                self.stop_program(name)
            if self.node:
                self.node.destroy_node()
                self.rclpy.shutdown()


def main():
    """@brief 使用独占锁避免两个客户端同时接管同一底盘。"""
    lock = open('/tmp/robot_console_gateway.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        emit('log', text='已有上位机连接，请先断开原连接')
        return
    gateway = Gateway()
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: setattr(gateway, 'running', False))
    gateway.run()


if __name__ == '__main__':
    main()
