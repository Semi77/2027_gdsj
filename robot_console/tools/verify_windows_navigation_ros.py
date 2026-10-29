"""隔离ROS命名空间验证Windows选点接口，不创建串口或硬件驱动。"""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    """@brief 使用模拟导航动作服务验证定位、目标接受和成功停稳，仅测试消息链路。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.action import ActionServer,ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from pi.navigation_control import NavigationControl
    from pi.gateway import Gateway
    events=[];initials=[];goals=[]

    class TestNode(Node):
        def __init__(self):
            """@brief 创建无任何硬件访问能力的测试底盘节点。"""
            super().__init__('windows_navigation_ros_check')
            self.declare_parameter('yaw_enabled',True)
            self.enabled=False;self.control=SimpleNamespace(enabled=True)
            self.status={'speeds':[0.,0.,0.,0.]};self.last_feedback=time.monotonic()

        def arm(self):
            """@brief 仅修改测试标志，不发送电机命令。"""
            self.enabled=True

        def disarm(self):
            """@brief 仅清除测试标志，不访问硬件。"""
            self.enabled=False

        def command(self, message):
            """@brief 所有测试输入必须保持零速，禁止意外测试非零运动。"""
            assert message.twist.linear.x==0 and message.twist.angular.z==0

    rclpy.init();node=TestNode();server_node=Node('windows_navigation_isolated_server')
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(server_node)
    def execute(handle):
        """@brief 模拟导航成功，只记录目标而不进行任何物理移动。"""
        pose=handle.request.pose
        goals.append([pose.header.frame_id,pose.pose.position.x,pose.pose.position.y])
        handle.succeed();return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,'/windows_nav_test/navigate_to_pose',execute)
    gateway=Gateway.__new__(Gateway);gateway.node=node;gateway.disarm=node.disarm
    navigation=NavigationControl(gateway,lambda *args,**kwargs:events.append(kwargs));gateway.navigation=navigation
    navigation.client.destroy()
    navigation.client=ActionClient(node,NavigateToPose,'/windows_nav_test/navigate_to_pose')
    navigation.cancel=node.create_client(CancelGoal,'/windows_nav_test/navigate_to_pose/_action/cancel_goal')
    navigation.initial_publisher=node.create_publisher(PoseWithCovarianceStamped,'/windows_nav_test/initialpose',1)
    subscription=server_node.create_subscription(PoseWithCovarianceStamped,'/windows_nav_test/initialpose',initials.append,1)
    navigation.readiness=lambda:''
    gateway.current_map={'frame':'map','width':20,'height':20,'resolution':.1,'origin':[-1.,-1.,0.], 'data':[0]*400}
    try:
        end=time.monotonic()+5
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.05)
            if navigation.client.server_is_ready() and navigation.initial_publisher.get_subscription_count()>0:break
        assert navigation.client.server_is_ready()
        gateway.map_pose_command({'action':'navigation_initial_pose','deadline':time.time()+1.,'x':.1,'y':.2,'yaw':.3})
        end=time.monotonic()+2
        while not initials and time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        assert initials and initials[0].header.frame_id=='map'
        navigation.enable()
        gateway.map_pose_command({'action':'navigation_goal','deadline':time.time()+1.,'x':.3,'y':.2,'yaw':.4})
        end=time.monotonic()+5
        while navigation.active and time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        assert goals==[['map',.3,.2]],goals
        assert navigation.result=='到达目标，等待停稳',navigation.result
        for _ in range(10):node.last_feedback=time.monotonic();navigation.tick()
        assert navigation.result=='已到达并停稳'
        print(json.dumps({'test_only_no_hardware':True,'initial_pose_received':len(initials),
            'goal_received':goals,'result':navigation.result,'hardware_ports_opened':False},ensure_ascii=False))
    finally:
        server.destroy();navigation.client.destroy();node.destroy_node();server_node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
