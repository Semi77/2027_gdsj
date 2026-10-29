"""在树莓派验证地图定位与只规划路径，全程保持车轮禁能。"""
import contextlib
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    """@brief 使用刚才SLAM记录的位置初始化定位，只计算路径，不请求行驶。"""
    from pi.gateway import Gateway
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    folder=Path('/workspace/ros2_ws/robot_console/logs')
    report={}
    with (folder/'navigation_static_check.log').open('w') as log, contextlib.redirect_stdout(log):
        gateway=Gateway()
        node=gateway.node
        try:
            gateway.open_serial('/dev/ttyAMA3')
            gateway.start_program('lidar')
            gateway.start_program('navigation')
            initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
            maps=[]
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscription=node.create_subscription(OccupancyGrid,'/map',maps.append,qos)
            started=time.monotonic();last_initial=0.0
            while time.monotonic()-started<50:
                gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
                if not gateway.navigation.pose_valid and time.monotonic()-last_initial>2:
                    msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
                    msg.header.stamp=node.get_clock().now().to_msg()
                    # 这是停止建图前实际读取的map到laser位姿，雷达偏移按用户约定为零。
                    msg.pose.pose.position.x=.007922779889331572
                    msg.pose.pose.position.y=.0015365190033324577
                    msg.pose.pose.orientation.z=.005736010326360364
                    msg.pose.pose.orientation.w=.9999835489574496
                    for index,value in ((0,.04),(7,.04),(35,.07)):msg.pose.covariance[index]=value
                    initial.publish(msg);last_initial=time.monotonic()
                if not gateway.navigation.readiness() and maps:
                    break
            report['readiness']=gateway.navigation.readiness()
            report['nodes']=node.get_node_names()
            report['host_enabled']=node.enabled
            report['firmware']=node.status.get('firmware')
            report['wheel_speeds']=node.status.get('speeds')
            report['crc_errors']=node.decoder.errors
            report['navigation_active']=gateway.navigation.active
            if not report['readiness']:
                transform=gateway.navigation.buffer.lookup_transform('map','base_link',__import__('rclpy').time.Time())
                x,y=transform.transform.translation.x,transform.transform.translation.y
                grid=maps[-1];resolution=grid.info.resolution
                def clear(px,py):
                    """@brief 验证候选目标附近20厘米区域均在地图已知空闲区。"""
                    cx=int((px-grid.info.origin.position.x)/resolution)
                    cy=int((py-grid.info.origin.position.y)/resolution)
                    radius=math.ceil(.20/resolution)
                    for dy in range(-radius,radius+1):
                        for dx in range(-radius,radius+1):
                            if dx*dx+dy*dy>radius*radius:continue
                            xx,yy=cx+dx,cy+dy
                            if not (0<=xx<grid.info.width and 0<=yy<grid.info.height):return False
                            if not 0<=grid.data[yy*grid.info.width+xx]<20:return False
                    return True
                candidates=[(x+.30*math.cos(a*math.pi/4),y+.30*math.sin(a*math.pi/4)) for a in range(8)]
                goal=next((p for p in candidates if clear(*p)),None)
                if goal:
                    client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
                    request=ComputePathToPose.Goal();request.goal=PoseStamped()
                    request.goal.header.frame_id='map';request.goal.header.stamp=node.get_clock().now().to_msg()
                    request.goal.pose.position.x,request.goal.pose.position.y=goal
                    request.goal.pose.orientation.w=1.0
                    request.planner_id='GridBased';request.use_start=False
                    future=client.send_goal_async(request);deadline=time.monotonic()+15
                    while not future.done() and time.monotonic()<deadline:
                        gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
                    if future.done() and future.result().accepted:
                        result=future.result().get_result_async()
                        while not result.done() and time.monotonic()<deadline:
                            gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
                        if result.done():
                            report['planning_status']=result.result().status
                            report['path_poses']=len(result.result().result.path.poses)
                            report['planning_goal']=goal
                else:report['planning_skip']='No nearby target with 20cm clearance'
                gateway.start_program('navigation_rviz')
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
                report['rviz_running']=gateway.processes['navigation_rviz'][0].poll() is None
                # 仅验证自主模式的零速使能握手，绝不发送非零速度或行驶目标。
                gateway.navigation.enable()
                deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
                report['zero_mode_enabled']=gateway.navigation.active and node.status.get('armed')
                report['zero_mode_targets']=node.status.get('targets')
                gateway.navigation.stop('零速握手验证完成，已禁能')
                deadline=time.monotonic()+.4
                while time.monotonic()<deadline:
                    gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.01)
            report['last_wheel_speeds']=node.status.get('speeds')
            report['last_armed']=node.status.get('armed')
        except Exception as error:
            report['error']=repr(error)
        finally:
            gateway.disarm()
            for name in list(gateway.processes):gateway.stop_program(name)
            node.close_port();node.destroy_node();gateway.rclpy.shutdown()
    (folder/'navigation_static_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
