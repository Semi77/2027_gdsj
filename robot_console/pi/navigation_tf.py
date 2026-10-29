"""导航模式专用的车体坐标发布，不与纯雷达建图同时运行。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class NavigationTF(Node):
    def __init__(self):
        """@brief 按初始假设发布中心安装雷达，转发真实里程计位姿。"""
        super().__init__('navigation_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
        transform.transform.rotation.w = 1.0
        self.static.sendTransform(transform)
        self.create_subscription(Odometry, '/odom', self.odometry, 10)

    def odometry(self, msg):
        """@brief 将真实odom消息转换为同时间戳的odom到base_link变换。"""
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    """@brief 随导航启动和停止坐标节点，保持主循环只处理ROS回调。"""
    rclpy.init()
    node = NavigationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
