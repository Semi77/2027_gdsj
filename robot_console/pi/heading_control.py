"""树莓派航向外环与轮速分配，纯计算模块便于离线验证。"""
import math


class HeadingControl:
    """使用左转递增的 yaw 保持直线，输出 AB/CD 四轮速度。"""

    def __init__(self):
        """@brief 初始化低增益参数及控制状态。"""
        self.kp, self.ki, self.kd = .010, .0, .002
        self.max_wz, self.track = .08, .130
        self.enabled = True
        self.reset()

    def reset(self):
        """@brief 停止、禁能和模式切换时清除锁定及积分。"""
        self.direction = 0
        self.target = self.previous = self.integral = self.rate = 0.
        self.correction = 0.

    @staticmethod
    def wrap(angle):
        """@brief 将 angle 指定的角差折算到正负 180 度。"""
        return (angle + 180.) % 360. - 180.

    def update(self, vx, wz, yaw, dt):
        """@brief 输入 m/s、rad/s、deg 和秒，返回 A/B/C/D 轮速 m/s。"""
        if not all(math.isfinite(v) for v in (vx, wz, yaw, dt)) or not 0 < dt <= .2:
            self.reset()
            raise ValueError('航向输入或采样间隔无效')
        correction = 0.
        direction = 1 if vx > 0 else -1
        if not self.enabled or abs(vx) < .001 or abs(wz) > .0005:
            self.reset()
        elif self.direction != direction:
            self.reset()
            self.direction, self.target, self.previous = direction, yaw, yaw
        else:
            error = self.wrap(self.target - yaw)
            alpha = dt / (.09 + dt)
            self.rate += alpha * (self.wrap(yaw-self.previous)/dt-self.rate)
            self.previous = yaw
            integral = max(-20., min(20., self.integral+error*dt)) if self.ki else 0.
            raw = self.kp*error + self.ki*integral - self.kd*self.rate
            if abs(raw) <= self.max_wz or raw*error < 0:
                self.integral = integral
            correction = max(-self.max_wz, min(self.max_wz, raw))
        self.correction = correction
        turn = wz + correction
        # 按用户实车确认：左转 AB 后退、CD 前进，倒车不反转纠偏符号。
        ab, cd = vx-turn*self.track/2, vx+turn*self.track/2
        scale = min(1., .3/max(abs(ab), abs(cd))) if ab or cd else 1.
        return [ab*scale, ab*scale, cd*scale, cd*scale]
