"""离线验证导航授权、超时停车、速度限制及停止确认。"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from concurrent.futures import Future
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def load_control():
    """@brief 替代ROS消息类型，保留自主控制的实际判断逻辑。"""
    names = ('action_msgs.msg','action_msgs.srv','geometry_msgs.msg','rclpy.action',
             'rclpy.time','nav2_msgs.action','tf2_ros')
    modules = {name:types.ModuleType(name) for name in names}
    for name,attrs in {'action_msgs.msg':['GoalStatusArray'], 'action_msgs.srv':['CancelGoal'],
        'geometry_msgs.msg':['Twist','PoseWithCovarianceStamped'], 'rclpy.action':['ActionClient'],
        'nav2_msgs.action':['NavigateToPose'], 'tf2_ros':['Buffer','TransformListener']}.items():
        for attr in attrs:setattr(modules[name],attr,object)
    modules['tf2_ros'].TransformException = Exception
    modules['rclpy.time'].Time = lambda nanoseconds=0:types.SimpleNamespace(to_msg=lambda:nanoseconds)
    modules['geometry_msgs.msg'].TwistStamped = lambda:types.SimpleNamespace(
        header=types.SimpleNamespace(),twist=types.SimpleNamespace(linear=types.SimpleNamespace(),angular=types.SimpleNamespace()))
    with patch.dict(sys.modules,modules):
        spec=importlib.util.spec_from_file_location('navigation_test_subject',Path(__file__).resolve().parents[1]/'pi/navigation_control.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 加载独立的实际导航控制模块。"""
        cls.module=load_control()

    def setUp(self):
        """@brief 构造未连接硬件的记录节点，使用固定时间验证命令租约。"""
        self.control=self.module.NavigationControl.__new__(self.module.NavigationControl)
        self.calls=[]
        node=types.SimpleNamespace(enabled=True,control=types.SimpleNamespace(enabled=True),
            last_feedback=10.0,status={'speeds':[0.0]*4})
        node.get_parameter=lambda _:types.SimpleNamespace(value=True)
        node.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=10_000_000_000,to_msg=lambda:10_000_000_000))
        node.command=self.calls.append
        node.disarm=lambda:setattr(node,'enabled',False)
        node.arm=lambda:setattr(node,'enabled',True)
        self.control.node=node
        self.control.gateway=types.SimpleNamespace(disarm=node.disarm)
        self.control.cancel=types.SimpleNamespace(service_is_ready=lambda:False)
        self.control.active=True;self.control.velocity=(0.05,0.1)
        self.control.last_command=9.9;self.control.goal_seen=False;self.control.goals_active=False
        self.control.command_pending=True;self.control.command_stamp_ns=9_900_000_000
        self.control.goal_pending=False;self.control.goal_generation=0;self.control.distance_remaining=None
        self.control.tracked_goal_id=None
        self.control.stopped_samples=0;self.control.result='导航中'
        self.control.readiness=lambda:''
        self.control.emit=lambda *args,**kwargs:None
        node.get_logger=lambda:types.SimpleNamespace(warning=lambda *args:None)

    def test_stale_motion_stops(self):
        """@brief 非零速度过期立即禁能，且不发送翻新的旧命令。"""
        with patch.object(self.module.time,'monotonic',return_value=10.2):self.control.tick()
        self.assertFalse(self.control.active);self.assertFalse(self.control.node.enabled)
        self.assertEqual(self.calls,[])

    def test_motion_preserves_original_stamp(self):
        """@brief 转发保留原接收时刻，不能把旧速度续期。"""
        with patch.object(self.module.time,'monotonic',return_value=10.0):self.control.tick()
        self.assertAlmostEqual(self.calls[0].header.stamp,9_900_000_000,delta=1)

    def test_nonzero_command_is_forwarded_only_once(self):
        """@brief 一条Nav2非零速度只转发一次，避免重复时间戳被底盘拒绝。"""
        with patch.object(self.module.time,'monotonic',return_value=10.0):
            self.control.tick();self.control.tick()
        self.assertEqual(len(self.calls),1)

    def test_no_rearm_old_goal(self):
        """@brief 已有旧目标时拒绝重新授权，避免恢复旧任务。"""
        self.control.active=False;self.control.goals_active=True
        with self.assertRaises(ValueError):self.control.enable()

    def test_invalid_velocity_stops(self):
        """@brief 拒绝侧向速度和超出导航限速的输入。"""
        msg=types.SimpleNamespace(linear=types.SimpleNamespace(x=.09,y=0.,z=0.),
                                 angular=types.SimpleNamespace(x=0.,y=0.,z=0.))
        self.control.command(msg)
        self.assertFalse(self.control.active)

    def test_arrival_waits_for_stopped_feedback(self):
        """@brief 导航成功后仍须连续新鲜零轮速才能报告停稳。"""
        self.control.active=False;self.control.result='到达目标，等待停稳'
        with patch.object(self.module.time,'monotonic',return_value=10.0):
            for _ in range(9):self.control.tick()
            self.assertEqual(self.control.result,'到达目标，等待停稳')
            self.control.tick()
        self.assertEqual(self.control.result,'已到达并停稳')

    def prepare_goal_rpc(self):
        """@brief 用可控异步响应代替ROS传输，测试实际目标生命周期回调。"""
        def pose():
            return types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
                orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        self.module.NavigateToPose=types.SimpleNamespace(Goal=lambda:types.SimpleNamespace(
            pose=types.SimpleNamespace(header=types.SimpleNamespace(),pose=pose())))
        self.module.PoseWithCovarianceStamped=lambda:types.SimpleNamespace(header=types.SimpleNamespace(),
            pose=types.SimpleNamespace(pose=pose(),covariance=[0.]*36))
        self.response=Future();self.result=Future();self.goals=[];self.cancelled=[];self.initials=[]
        def send(goal,feedback_callback):
            self.goals.append(goal);self.feedback=feedback_callback;return self.response
        self.control.client=types.SimpleNamespace(send_goal_async=send)
        self.control.initial_publisher=types.SimpleNamespace(get_subscription_count=lambda:1,publish=self.initials.append)
        self.handle=types.SimpleNamespace(accepted=True,goal_id=types.SimpleNamespace(uuid=[2]*16),get_result_async=lambda:self.result,
                                         cancel_goal_async=lambda:self.cancelled.append(True))

    def test_goal_accept_and_success_result(self):
        """@brief 验证地图目标直接进入导航动作，真实成功响应撤销使能并等待停稳。"""
        self.prepare_goal_rpc();self.control.submit_goal({'x':.2,'y':.1,'yaw':.3})
        self.assertTrue(self.control.goal_pending)
        self.assertEqual(self.goals[0].pose.header.frame_id,'map')
        self.assertAlmostEqual(self.goals[0].pose.pose.position.x,.2)
        self.response.set_result(self.handle)
        self.assertFalse(self.control.goal_pending);self.assertTrue(self.control.goal_seen)
        self.result.set_result(types.SimpleNamespace(status=4))
        self.assertFalse(self.control.active);self.assertFalse(self.control.node.enabled)
        self.assertEqual(self.control.result,'到达目标，等待停稳')

    def test_late_accept_after_cancel_is_cancelled(self):
        """@brief 取消后晚到的目标接受响应不能重新启动车轮。"""
        self.prepare_goal_rpc();self.control.submit_goal({'x':.2,'y':.1,'yaw':.3})
        self.control.stop('操作员取消');self.response.set_result(self.handle)
        self.assertEqual(self.cancelled,[True]);self.assertFalse(self.control.active)
        self.assertEqual(self.control.result,'操作员取消')

    def test_goal_without_authorization_is_rejected(self):
        """@brief 定位正常也不能在未允许自主控制时提交目标。"""
        self.prepare_goal_rpc();self.control.active=False
        with self.assertRaises(ValueError):self.control.submit_goal({'x':.2,'y':.1,'yaw':.3})
        self.assertEqual(self.goals,[])

    def test_initial_pose_requires_stopped_chassis(self):
        """@brief 初始位置仅在停车时发布，朝向按原弧度值构造四元数。"""
        self.prepare_goal_rpc()
        with self.assertRaises(ValueError):self.control.set_initial_pose({'x':.2,'y':.1,'yaw':.3})
        self.control.active=False;self.control.node.enabled=False
        self.control.set_initial_pose({'x':.2,'y':.1,'yaw':.3})
        self.assertEqual(len(self.initials),1)
        self.assertEqual(self.initials[0].header.frame_id,'map')
        self.assertAlmostEqual(self.initials[0].pose.pose.orientation.z,__import__('math').sin(.15))

    def test_late_feedback_cannot_restore_cancelled_goal(self):
        """@brief 取消后的旧目标反馈不再改变剩余距离或状态。"""
        self.prepare_goal_rpc();self.control.submit_goal({'x':.2,'y':.1,'yaw':.3})
        self.control.stop('操作员取消')
        self.feedback(types.SimpleNamespace(feedback=types.SimpleNamespace(distance_remaining=.1)))
        self.assertIsNone(self.control.distance_remaining)

    def test_old_goal_status_does_not_cancel_new_goal(self):
        """@brief 服务器缓存的旧任务终态不能撤销当前已接受的新目标。"""
        self.prepare_goal_rpc();self.control.submit_goal({'x':.2,'y':.1,'yaw':.3})
        self.response.set_result(self.handle)
        status=types.SimpleNamespace(status=4,goal_info=types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=[1]*16),stamp=types.SimpleNamespace(sec=1,nanosec=0)))
        self.control.goal_status(types.SimpleNamespace(status_list=[status]))
        self.assertTrue(self.control.active);self.assertTrue(self.control.node.enabled)


if __name__=='__main__':unittest.main()
