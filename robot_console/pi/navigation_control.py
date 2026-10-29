"""将Nav2速度接入现有底盘，集中处理模式互斥、超时和取消。"""
import math
import time
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from shared.navigation_geometry import pose_values
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


class NavigationControl:
    def __init__(self, gateway, emit):
        """@brief gateway提供独占底盘节点，emit向上位机输出真实导航状态。"""
        self.gateway, self.node, self.emit = gateway, gateway.node, emit
        self.active = False
        self.last_command = self.last_scan = 0.0
        self.command_pending = False
        self.command_stamp_ns = 0
        self.velocity = (0.0, 0.0)
        self.pose_valid = False
        self.goals_active = False
        self.goal_seen = False
        self.result = '未启动自主控制'
        self.stopped_samples = 0
        self.goal_pending = False
        self.goal_generation = 0
        self.tracked_goal_id = None
        self.goal_sent_at = 0.0
        self.distance_remaining = None
        self.initial_publisher = self.node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.cancel = self.node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.subscriptions = [
            self.node.create_subscription(Twist, '/nav_cmd_vel', self.command, 1),
            self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose, 10),
            self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status, 10),
        ]

    def pose(self, msg):
        """@brief 接收AMCL定位质量，不把手工初始位置直接视为定位成功。"""
        cov = msg.pose.covariance
        self.pose_valid = (msg.header.frame_id == 'map' and
            all(math.isfinite(cov[i]) and 0 <= cov[i] < 0.25 for i in (0,7,35)))

    def readiness(self):
        """@brief 检查导航服务、雷达、地图坐标及底盘反馈是否可用。"""
        now = time.monotonic()
        if not self.node.serial or now-self.node.last_feedback > 0.2:
            return '请先打开底盘串口并等待反馈'
        if not self.client.server_is_ready():
            return '等待Nav2导航服务启动'
        if not self.pose_valid:
            return '请在地图上拖动设置初始位置，并等待定位收敛'
        if now-self.last_scan > 0.5:
            return '雷达数据超时'
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
            age = self.node.get_clock().now().nanoseconds/1e9-stamp
            if not -0.6 <= age <= 0.5:
                return '地图定位或里程计坐标超时'
        except TransformException:
            return '等待map→odom→base_link坐标链'
        return ''

    def enable(self):
        """@brief 显式授权自主模式，只在无旧目标且定位正常时零速使能。"""
        reason = self.readiness()
        if reason:
            raise ValueError(reason)
        if self.goals_active:
            raise ValueError('请先取消原导航目标，等待取消完成后再允许自主控制')
        self.gateway.disarm()
        self.node.control.enabled = False
        try:
            self.node.arm()
        except Exception:
            self.node.control.enabled = bool(self.node.get_parameter('yaw_enabled').value)
            raise
        self.active = True
        self.goal_seen = False
        self.tracked_goal_id = None
        self.velocity = (0.0,0.0)
        self.last_command = time.monotonic()
        self.command_pending = False
        self.result = '自主控制已允许，请在地图上拖动设置导航目标'
        self.emit('log', text=self.result)

    def stop(self, reason='已取消并禁能', cancel=True):
        """@brief 立即禁能底盘，再异步取消目标，不等待网络返回才停车。"""
        self.active = False
        self.velocity = (0.0,0.0)
        self.command_pending = False
        self.goal_pending = False
        self.goal_generation += 1
        self.distance_remaining = None
        self.node.disarm()
        self.node.control.enabled = bool(self.node.get_parameter('yaw_enabled').value)
        self.result = reason
        self.stopped_samples = 0
        self.emit('log', text='自主控制停止：'+reason)
        self.node.get_logger().warning('自主控制停止：'+reason)
        if cancel and self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())

    def set_initial_pose(self, data):
        """@brief data包含map坐标的x、y、yaw；只在停车且无活动任务时更新定位。"""
        x,y,yaw=pose_values(data)
        if self.active or self.node.enabled or self.goals_active or self.goal_pending:
            raise ValueError('请先取消导航并停车，再设置初始位置')
        if self.initial_publisher.get_subscription_count()==0:
            raise ValueError('AMCL尚未就绪，请先启动地图定位导航')
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y=x,y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        for index,value in ((0,.04),(7,.04),(35,.06854)):msg.pose.covariance[index]=value
        self.pose_valid=False
        self.initial_publisher.publish(msg)
        self.result='初始位置已提交，等待雷达与地图匹配'

    def submit_goal(self, data):
        """@brief 将Windows选定的x、y、yaw直接提交NavigateToPose，保留授权与定位检查。"""
        x,y,yaw=pose_values(data)
        if not self.active:raise ValueError('请先点击允许自主控制')
        reason=self.readiness()
        if reason:raise ValueError(reason)
        if self.goals_active or self.goal_pending:raise ValueError('已有导航任务，请先取消或等待完成')
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.header.stamp=self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=x,y
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.goal_generation+=1;generation=self.goal_generation
        self.goal_pending=True;self.goal_sent_at=time.monotonic();self.distance_remaining=None
        self.result='目标已发送，等待Nav2接受'
        future=self.client.send_goal_async(goal,feedback_callback=lambda m:self.goal_feedback(m,generation))
        future.add_done_callback(lambda f:self.goal_response(f,generation))

    def goal_feedback(self, message, generation):
        """@brief 读取真实导航剩余距离，非有限数据不进入界面。"""
        if generation!=self.goal_generation or not self.active:return
        value=message.feedback.distance_remaining
        self.distance_remaining=float(value) if math.isfinite(value) and value>=0 else None

    def goal_response(self, future, generation):
        """@brief 处理导航接受或拒绝，取消后晚到的接受响应立即撤销。"""
        try:
            handle=future.result()
            if generation!=self.goal_generation or not self.active:
                if handle.accepted:handle.cancel_goal_async()
                return
            self.goal_pending=False
            if not handle.accepted:
                self.stop('Nav2拒绝目标，已停车');return
            self.tracked_goal_id=bytes(handle.goal_id.uuid)
            self.goal_seen=True;self.result='导航中'
            self.emit('log',text='Nav2已接受Windows地图目标')
            handle.get_result_async().add_done_callback(lambda f:self.goal_result(f,generation))
        except Exception as error:
            if generation==self.goal_generation:self.stop('目标提交失败：'+str(error))

    def goal_result(self, future, generation):
        """@brief 处理真实动作结果，导航成功后仍由轮速反馈确认停稳。"""
        if generation!=self.goal_generation or not self.active:return
        try:
            self.stop('到达目标，等待停稳' if future.result().status==4 else '导航取消或失败，已禁能',cancel=False)
        except Exception as error:self.stop('导航结果异常：'+str(error))

    def command(self, msg):
        """@brief 接收Nav2专用速度，禁止侧移、非有限值和超出低速限制的请求。"""
        if not self.active:
            return
        vx,wz = msg.linear.x,msg.angular.z
        other = (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)
        if (not all(math.isfinite(v) for v in (vx,wz,*other)) or
            any(v != 0 for v in other) or abs(vx)>0.0801 or abs(wz)>0.4001):
            self.stop('Nav2速度无效或超限，已停车')
            return
        self.velocity = (vx,wz)
        self.last_command = time.monotonic()
        self.command_stamp_ns = self.node.get_clock().now().nanoseconds
        self.command_pending = True

    def goal_status(self, msg):
        """@brief 跟踪导航动作的终态，成功后仍需反馈轮速确认停稳。"""
        self.goals_active = any(s.status in (1,2,3) for s in msg.status_list)
        if not self.active:
            return
        # 只处理当前目标；动作服务器保留的旧任务终态不能取消新任务。
        if self.goal_pending:return
        if self.tracked_goal_id is None:
            candidates=[s for s in msg.status_list if s.status in (1,2,3)]
            if not candidates:return
            newest=max(candidates,key=lambda s:(s.goal_info.stamp.sec,s.goal_info.stamp.nanosec))
            self.tracked_goal_id=bytes(newest.goal_info.goal_id.uuid)
        statuses=[s for s in msg.status_list if bytes(s.goal_info.goal_id.uuid)==self.tracked_goal_id]
        if not statuses:return
        newest=max(statuses,key=lambda s:(s.goal_info.stamp.sec,s.goal_info.stamp.nanosec))
        if newest.status in (1,2,3):
            self.goal_seen = True
            self.result = '导航中'
        elif self.goal_seen:
            self.stop('到达目标，等待停稳' if newest.status == 4 else '导航取消或失败，已禁能', cancel=False)

    def tick(self):
        """@brief 每轮只转发新鲜速度；等待目标时仅保活零速，异常不自动重启。"""
        if not self.active:
            if self.result == '到达目标，等待停稳':
                fresh = time.monotonic()-self.node.last_feedback < 0.2
                stopped = all(abs(v)<0.015 for v in self.node.status.get('speeds',[1]*4))
                self.stopped_samples = self.stopped_samples+1 if fresh and stopped else 0
                if self.stopped_samples >= 10:
                    self.result = '已到达并停稳'
            return
        reason = self.readiness()
        if reason or not self.node.enabled:
            self.stop(reason or '底盘已撤销使能，需重新授权')
            return
        if self.goal_pending and time.monotonic()-self.goal_sent_at>3:
            self.stop('Nav2目标响应超时，已停车');return
        if time.monotonic()-self.last_command > 0.2:
            if any(v != 0 for v in self.velocity):
                self.stop('导航速度超时，已停车')
                return
            self.velocity = (0.0,0.0)
        msg = TwistStamped()
        if self.command_pending:
            stamp = self.command_stamp_ns
            self.command_pending = False
        elif any(self.velocity):
            return
        else:
            # 等待目标时只允许新鲜零速保活；非零输入不在循环中重发。
            stamp = self.node.get_clock().now().nanoseconds
        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x,msg.twist.angular.z = self.velocity
        self.node.command(msg)

    def status(self):
        """@brief 输出授权、定位检查和任务结果，不以进程存活代替就绪。"""
        reason = self.readiness()
        return {'active':self.active, 'ready':not reason, 'reason':reason, 'text':self.result,
                'distance_remaining':self.distance_remaining, 'goal_pending':self.goal_pending}
