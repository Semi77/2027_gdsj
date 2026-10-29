"""实时图形组件，仅展示真实输入；空数据状态显示提示。"""
from collections import deque
import math
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF, QImage
from PySide6.QtWidgets import QWidget

COLORS = ['#52d6c7', '#66a9ff', '#ffbd69', '#d5a3ff']


class SpeedPlot(QWidget):
    def __init__(self):
        """@brief 创建固定长度轮速曲线，限制长期运行内存占用。"""
        super().__init__()
        self.samples = deque(maxlen=600)
        self.setMinimumHeight(260)

    def append(self, stamp, values):
        """@brief 添加以秒和 m/s 表示的四轮反馈采样。"""
        self.samples.append((stamp, values))
        self.update()

    def paintEvent(self, event):
        """@brief 绘制最近十秒实测轮速，缺测不补造数值。"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(55, 30, max(10,self.width()-80), max(10,self.height()-65))
        painter.fillRect(self.rect(), QColor('#111c2b'))
        bound = max(.2, max((abs(v) for _, values in self.samples for v in values), default=0)*1.2)
        painter.setPen(QColor('#334456'))
        for i in range(5):
            y = rect.top()+rect.height()*i/4
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.drawText(QRectF(0,y-10,49,20), Qt.AlignRight, f'{bound*(1-i/2):.2f}')
        painter.setPen(QColor('#9badc3'))
        painter.drawText(12,18,'m/s')
        painter.drawText(int(rect.right()-85),self.height()-10,'最近 10 秒')
        if not self.samples:
            painter.drawText(rect, Qt.AlignCenter, '等待四轮速度反馈')
            return
        now = self.samples[-1][0]
        for wheel in range(4):
            painter.setPen(QPen(QColor(COLORS[wheel]), 2))
            previous = None
            previous_time = None
            for t, values in self.samples:
                if t < now-10:
                    continue
                point = QPointF(rect.right()-(now-t)/10*rect.width(),
                                rect.center().y()-values[wheel]/bound*rect.height()/2)
                if previous is not None and t-previous_time < .3:
                    painter.drawLine(previous,point)
                previous, previous_time = point, t


class Attitude(QWidget):
    def __init__(self):
        """@brief 初始化姿态示意图，未接入时不显示虚构姿态。"""
        super().__init__()
        self.angles = None
        self.setMinimumHeight(260)

    def paintEvent(self,event):
        """@brief 按输入的横滚、俯仰、航向角绘制车体线框。"""
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#111c2b'))
        p.setPen(QColor('#9badc3'))
        if self.angles is None:
            p.drawText(self.rect(),Qt.AlignCenter,'等待有效 IMU 姿态')
            return
        roll,pitch,yaw = [math.radians(v) for v in self.angles]
        points=[]
        scale=min(self.width()/5,self.height()/4)
        for x,y,z in [(-1,-.6,-.2),(1,-.6,-.2),(1,.6,-.2),(-1,.6,-.2),
                      (-1,-.6,.2),(1,-.6,.2),(1,.6,.2),(-1,.6,.2)]:
            y,z=y*math.cos(roll)-z*math.sin(roll),y*math.sin(roll)+z*math.cos(roll)
            x,z=x*math.cos(pitch)+z*math.sin(pitch),-x*math.sin(pitch)+z*math.cos(pitch)
            x,y=x*math.cos(yaw)-y*math.sin(yaw),x*math.sin(yaw)+y*math.cos(yaw)
            points.append(QPointF(self.width()/2+(x-y*.4)*scale,self.height()/2+(-z+y*.45+x*.2)*scale))
        p.setPen(QPen(QColor('#52d6c7'),2))
        for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
            p.drawLine(points[a],points[b])
        p.setPen(QColor('#ffbd69'))
        p.drawText(points[5], '  车头 +X')


class SpatialView(QWidget):
    def __init__(self, map_mode=False):
        """@brief 地图和激光分别展示，避免缺少 TF 时错误叠加。"""
        super().__init__()
        self.map_mode = map_mode
        self.points = []
        self.image = None
        self.setMinimumHeight(350)

    def set_map(self, data):
        """@brief 从栅格消息生成灰度图，保持 ROS 地图纵轴方向。"""
        width,height = data['width'],data['height']
        if width <= 0 or height <= 0 or width*height != len(data['data']):
            return
        pixels=bytes(110 if v<0 else max(0,min(255,255-round(v*2.55))) for v in data['data'])
        self.image=QImage(pixels,width,height,width,QImage.Format_Grayscale8).copy().mirrored(False,True)
        self.update()

    def paintEvent(self,event):
        """@brief 绘制地图或激光自身坐标点云，缺数据时显示明确提示。"""
        p=QPainter(self)
        p.fillRect(self.rect(),QColor('#111c2b'))
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QColor('#9badc3'))
        if self.map_mode:
            if self.image is None:
                p.drawText(self.rect(),Qt.AlignCenter,'等待 /map · 请先启动雷达与建图')
            else:
                size=self.image.size().scaled(self.size(),Qt.KeepAspectRatio)
                p.drawImage(QRectF((self.width()-size.width())/2,(self.height()-size.height())/2,size.width(),size.height()),self.image)
            return
        scale=min(self.width(),self.height())/9
        center=QPointF(self.width()/2,self.height()/2)
        p.setPen(QColor('#293d50'))
        for r in range(1,5):
            p.drawEllipse(center,r*scale,r*scale)
        p.drawText(10,22,'激光自身坐标 · 网格 1 m · 上方为 +X')
        if not self.points:
            p.drawText(self.rect(),Qt.AlignCenter,'等待 /scan')
        p.setPen(QPen(QColor('#52d6c7'),3))
        for x,y in self.points:
            p.drawPoint(QPointF(center.x()-y*scale,center.y()-x*scale))
