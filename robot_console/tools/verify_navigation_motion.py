"""经用户授权的20厘米低速导航验收，含独立距离、角度和时间边界。"""
import contextlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    """@brief 完成真实地图定位、导航动作及停稳验收，异常立即取消并禁能。"""
    from pi.gateway import Gateway
    from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from navigation_scan_match import match_scan
    folder=Path('/workspace/ros2_ws/robot_console/logs')
    report={'goal_distance_m':.20,'events':[],'motion_samples':[],'raw_commands':[],'smoothed_commands':[]}
    with (folder/'full_flow_check.log').open('w') as output,contextlib.redirect_stdout(output):
        gateway=Gateway();node=gateway.node;scan=[];maps=[]
        subscriptions=[node.create_subscription(LaserScan,'/scan',lambda m:scan.__setitem__(slice(None),[m]),qos_profile_sensor_data),
            node.create_subscription(OccupancyGrid,'/map',lambda m:maps.__setitem__(slice(None),[m]),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        for key,topic in [('raw_commands','/nav_cmd_vel_raw'),('smoothed_commands','/nav_cmd_vel')]:
            subscriptions.append(node.create_subscription(Twist,topic,
                lambda m,k=key:report[k].append([time.monotonic(),m.linear.x,m.angular.z]),10))
        original_disarm=node.disarm
        def disarm():
            """@brief 记录撤销使能时的真实上下文，便于区分超时、拒绝和任务终态。"""
            if node.enabled:
                now=time.monotonic()
                report['events'].append({'event':'disarm','stack':traceback.format_stack(limit=4),
                    'deadline_remaining':node.deadline-now,'feedback_age':now-node.last_feedback,
                    'ack_age':now-node.last_ack_time,'last_error':node.last_error,'status':dict(node.status)})
            original_disarm()
        node.disarm=disarm
        def spin(duration):
            """@brief 让控制循环与ROS回调持续运行，不用阻塞等待代替速度保活。"""
            end=time.monotonic()+duration
            while time.monotonic()<end:
                gateway.poll();gateway.rclpy.spin_once(node,timeout_sec=.005)
        try:
            gateway.open_serial('/dev/ttyAMA3');gateway.start_program('lidar');gateway.start_program('navigation')
            started=time.monotonic()
            while (not scan or not maps) and time.monotonic()-started<15:spin(.1)
            if not scan or not maps:raise RuntimeError('未收到雷达或地图')
            initial=match_scan(maps[-1],scan[-1]);report['scan_match']=initial
            publisher=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
            last_initial=0.0;started=time.monotonic()
            while time.monotonic()-started<35:
                spin(.1)
                if not gateway.navigation.pose_valid and time.monotonic()-last_initial>1.5:
                    msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.pose.pose.position.x,msg.pose.pose.position.y=initial['x'],initial['y']
                    msg.pose.pose.orientation.z=math.sin(initial['yaw']/2);msg.pose.pose.orientation.w=math.cos(initial['yaw']/2)
                    for i,v in [(0,.02),(7,.02),(35,.03)]:msg.pose.covariance[i]=v
                    publisher.publish(msg);last_initial=time.monotonic()
                if not gateway.navigation.readiness():break
            reason=gateway.navigation.readiness()
            if reason:raise RuntimeError(reason)
            front=[r for i,r in enumerate(scan[-1].ranges)
                   if math.isfinite(r) and abs((scan[-1].angle_min+i*scan[-1].angle_increment+math.pi)%(2*math.pi)-math.pi)<.25]
            report['front_clearance_m']=min(front) if front else None
            if not front or min(front)<.45:raise RuntimeError('雷达前方未确认45厘米净空')
            transform=gateway.navigation.buffer.lookup_transform('map','base_link',Time())
            p,q=transform.transform.translation,transform.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            request.pose.header.stamp=node.get_clock().now().to_msg()
            request.pose.pose.position.x=p.x+.20*math.cos(yaw)
            request.pose.pose.position.y=p.y+.20*math.sin(yaw)
            request.pose.pose.orientation=q
            report['start_map_pose']=[p.x,p.y,yaw]
            report['goal_map_pose']=[request.pose.pose.position.x,request.pose.pose.position.y,yaw]
            start_odom=(node.odometry.x,node.odometry.y);start_yaw=node.status['rpy_deg'][2]
            gateway.navigation.enable();spin(.25)
            if not gateway.navigation.active:raise RuntimeError(gateway.navigation.result)
            future=gateway.navigation.client.send_goal_async(request);started=time.monotonic()
            while not future.done() and time.monotonic()-started<3:spin(.02)
            if not future.done() or not future.result().accepted:raise RuntimeError('导航目标未被接受')
            result=future.result().get_result_async();report['goal_accepted']=True
            while not result.done() and time.monotonic()-started<15:
                spin(.05)
                distance=math.hypot(node.odometry.x-start_odom[0],node.odometry.y-start_odom[1])
                heading=abs((node.status['rpy_deg'][2]-start_yaw+180)%360-180)
                report['motion_samples'].append({'t':time.monotonic()-started,'distance':distance,
                    'speeds':node.status.get('speeds'),'targets':node.status.get('targets'),
                    'armed':node.status.get('armed'),'active':gateway.navigation.active,'text':gateway.navigation.result})
                if distance>.35 or heading>30:
                    gateway.navigation.stop('验收行程或转角超过边界');break
                recent_front=[r for i,r in enumerate(scan[-1].ranges) if math.isfinite(r) and
                    abs((scan[-1].angle_min+i*scan[-1].angle_increment+math.pi)%(2*math.pi)-math.pi)<.25]
                if recent_front and min(recent_front)<.20:
                    gateway.navigation.stop('验收前方障碍距离不足20厘米');break
                temp=float(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000
                if temp>=75:gateway.navigation.stop('验收温度达到75度');break
                if not gateway.navigation.active and gateway.navigation.result!='到达目标，等待停稳':break
            if result.done():report['action_status']=result.result().status
            else:gateway.navigation.stop('验收任务未完成，已取消')
            spin(.7)
            report['navigation_result']=gateway.navigation.result
            report['travelled_m']=math.hypot(node.odometry.x-start_odom[0],node.odometry.y-start_odom[1])
            report['max_wheel_speed']=max((max(map(abs,s['speeds'])) for s in report['motion_samples']),default=0)
            report['last_feedback']=node.status;report['crc_errors']=node.decoder.errors
        except Exception as error:
            report['error']=repr(error)
        finally:
            gateway.disarm();spin(.4)
            report['final_armed']=node.status.get('armed');report['final_speeds']=node.status.get('speeds')
            for name in list(gateway.processes):gateway.stop_program(name)
            node.close_port();node.destroy_node();gateway.rclpy.shutdown()
    (folder/'full_flow_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('motion_samples','raw_commands','smoothed_commands','events')},ensure_ascii=False))


if __name__=='__main__':main()
