"""累计编码器与 IMU 航向的平面里程计，不等同于外部定位。"""
import math


class WheelOdometry:
    def __init__(self, diameter=.048, counts_per_rev=1040.):
        """@brief diameter 为轮径m，counts_per_rev 为轮子每圈四倍频计数。"""
        self.diameter, self.counts_per_rev = diameter, counts_per_rev
        self.x = self.y = self.heading = 0.
        self.reset_reference()

    def reset_reference(self):
        """@brief 重连、重启或换IMU时只重建采样基准，保留已累计的局部位置。"""
        self.previous = None

    def update(self, counts, sample_ms, session, yaw):
        """@brief 根据uint32计数和MCU采样时间计算位移，yaw单位deg且左转为正。"""
        if len(counts) != 4 or not math.isfinite(yaw):
            self.reset_reference()
            return None
        current = (list(counts), sample_ms, session, yaw)
        old, self.previous = self.previous, current
        if old is None or session != old[2]:
            return None
        elapsed = (sample_ms-old[1]) & 0xffffffff
        if elapsed == 0:
            self.previous = old
            return None
        if elapsed > 1000:
            return None
        dt = elapsed/1000.
        scale = math.pi*self.diameter/self.counts_per_rev
        distances = [((a-b+0x80000000) % 0x100000000-0x80000000)*scale
                     for a,b in zip(counts,old[0])]
        # 超过3m/s视为计数突变；当前底盘指令上限远低于该值。
        if max(map(abs,distances)) > 3.*dt:
            return None
        distance = sum(distances)/4.
        rotation = math.radians((yaw-old[3]+180.) % 360.-180.)
        middle = self.heading+rotation/2.
        self.x += distance*math.cos(middle)
        self.y += distance*math.sin(middle)
        self.heading = (self.heading+rotation+math.pi) % (2*math.pi)-math.pi
        return {'x':self.x, 'y':self.y, 'yaw_rad':self.heading,
                'vx':distance/dt, 'wz':rotation/dt, 'dt':dt}


def quaternion_yaw(x, y, z, w):
    """@brief 校验姿态四元数并返回yaw角度，要求外部驱动已转换到车体坐标系。"""
    if not all(math.isfinite(v) for v in (x,y,z,w)):
        raise ValueError('外部IMU四元数无效')
    norm = x*x+y*y+z*z+w*w
    if not .9 <= norm <= 1.1:
        raise ValueError('外部IMU四元数未归一化')
    x,y,z,w = (v/math.sqrt(norm) for v in (x,y,z,w))
    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
