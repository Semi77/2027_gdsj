"""只传输二维坐标数据给Windows，不在树莓派创建图形窗口。"""
import math
import time
from rclpy.time import Time
from tf2_ros import TransformException
from shared.navigation_geometry import transform_points


def planar_transform(transform):
    """@brief 从真实TF提取平面x、y、yaw，拒绝非有限或未归一化四元数。"""
    p,q=transform.transform.translation,transform.transform.rotation
    values=(p.x,p.y,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or not .9<q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1.1:
        raise ValueError('坐标变换无效')
    return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


class NavigationScene:
    def __init__(self, gateway):
        """@brief gateway提供ROS节点、扫描缓存及TF缓存，不独占任何硬件。"""
        self.gateway=gateway;self.path=None;self.path_received=0.0
        from nav_msgs.msg import Path
        self.subscription=gateway.node.create_subscription(Path,'/plan',self.on_path,1)

    def on_path(self, msg):
        """@brief 缓存最新规划路径，数据量在传输时限制到600个点。"""
        self.path=msg;self.path_received=time.monotonic()

    def snapshot(self):
        """@brief 输出map坐标中的车体、扫描和路径，无TF时明确留空。"""
        gateway=self.gateway;node=gateway.node;navigation=gateway.navigation
        data={'frame':'map','pose':None,'laser':[],'path':[],'reason':'','server_time':time.time()}
        if not navigation:
            data['reason']='导航组件未安装，无法叠加坐标';return data
        now=node.get_clock().now().nanoseconds/1e9
        try:
            transform=navigation.buffer.lookup_transform('map','base_link',Time())
            stamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            if not -.6<=now-stamp<=.5:raise ValueError('车体坐标超时')
            data['pose']=planar_transform(transform)
        except (TransformException,ValueError):
            data['reason']='等待初始定位或车体坐标，暂不显示蓝色车体'
        scan=gateway.last_scan_message
        if scan is not None:
            stamp=scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
            if 0<=now-stamp<.6:
                try:
                    transform=navigation.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp))
                    points=[]
                    step=max(1,math.ceil(len(scan.ranges)/720))
                    for i in range(0,len(scan.ranges),step):
                        distance=scan.ranges[i]
                        if math.isfinite(distance) and scan.range_min<=distance<=scan.range_max:
                            angle=scan.angle_min+i*scan.angle_increment
                            points.append([distance*math.cos(angle),distance*math.sin(angle)])
                    data['laser']=transform_points(points,planar_transform(transform))
                except (TransformException,ValueError):
                    data['reason']=data['reason'] or '激光时间对应的坐标尚未就绪，暂不叠加红点'
            else:data['reason']=data['reason'] or '实时激光超时'
        else:data['reason']=data['reason'] or '等待实时激光'
        if self.path is not None and navigation.active and navigation.goal_seen and time.monotonic()-self.path_received<5:
            try:
                step=max(1,math.ceil(len(self.path.poses)/600))
                points=[[p.pose.position.x,p.pose.position.y] for p in self.path.poses[::step]]
                if not all(math.isfinite(v) for p in points for v in p):raise ValueError('路径无效')
                if self.path.header.frame_id!='map':
                    transform=navigation.buffer.lookup_transform('map',self.path.header.frame_id,Time.from_msg(self.path.header.stamp))
                    points=transform_points(points,planar_transform(transform))
                data['path']=points
            except (TransformException,ValueError):pass
        if data['pose']:data['pose']=[round(v,4) for v in data['pose']]
        for key in ('laser','path'):data[key]=[[round(v,4) for v in p] for p in data[key]]
        return data
