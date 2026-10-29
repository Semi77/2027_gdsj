"""Windows本地二维导航画布，负责绘制与选点，不执行导航算法。"""
import math
import time
from PySide6.QtCore import QPointF,QRectF,Qt,Signal,QTimer
from PySide6.QtGui import QColor,QImage,QPainter,QPen,QPolygonF
from PySide6.QtWidgets import QWidget
from shared.navigation_geometry import MapGeometry,transform_points


class NavigationView(QWidget):
    posePicked=Signal(str,float,float,float)
    modeChanged=Signal(str)
    notice=Signal(str)

    def __init__(self):
        """@brief 创建本地地图画布，默认拖动平移且不允许离线发送位置。"""
        super().__init__()
        self.map_geometry=None;self.cells=[];self.image=None
        self.center=QPointF(0,0);self.scale=150.0
        self.scene={};self.scene_received=0.0
        self.mode='pan';self.connected=False
        self.drag_start=None;self.drag_end=None;self.pan_start=None
        self.selected=None
        self.setMinimumSize(400,350)
        self.setMouseTracking(True)
        self.timer=QTimer(self);self.timer.timeout.connect(self.update);self.timer.start(200)

    def set_map(self, data):
        """@brief 缓存地图图像和真实原点信息，后续重绘不重复转换所有栅格。"""
        try:
            geometry=MapGeometry(data);cells=data['data']
            if len(cells)!=geometry.width*geometry.height or any(type(v) is not int or not -1<=v<=100 for v in cells):
                raise ValueError('地图栅格数据无效')
            pixels=bytes(110 if v<0 else 255-round(v*2.55) for v in cells)
            image=QImage(pixels,geometry.width,geometry.height,geometry.width,QImage.Format_Grayscale8).copy()
        except (ValueError,TypeError,KeyError) as error:
            self.notice.emit('地图未更新：'+str(error));return
        first=self.map_geometry is None
        self.map_geometry,self.cells,self.image=geometry,list(cells),image
        if first:self.fit_map()
        self.update()

    def set_scene(self, data, age=0.0):
        """@brief 接收已在树莓派完成坐标变换的真实叠加数据，超时后自动隐藏。"""
        self.scene=data;self.scene_received=time.monotonic()-max(0.0,age);self.update()

    def set_connected(self, connected):
        """@brief 断线清除叠加数据与选点模式，保留地图作为历史背景。"""
        self.connected=connected
        if not connected:
            self.scene={};self.selected=None;self.set_mode('pan')
        self.update()

    def set_mode(self, mode):
        """@brief mode为pan、initial或goal，切换时清除未提交的拖动。"""
        if mode not in ('pan','initial','goal'):raise ValueError('未知地图操作模式')
        self.mode=mode;self.drag_start=self.drag_end=None
        self.setCursor(Qt.ArrowCursor if mode=='pan' else Qt.CrossCursor)
        self.modeChanged.emit(mode);self.update()

    def fit_map(self):
        """@brief 根据地图旋转后的边界自动居中并适配当前窗口。"""
        if not self.map_geometry:return
        g=self.map_geometry
        corners=[g.cell_to_world(x,y) for x,y in ((0,0),(g.width,0),(g.width,g.height),(0,g.height))]
        xs,ys=zip(*corners)
        self.center=QPointF((min(xs)+max(xs))/2,(min(ys)+max(ys))/2)
        self.scale=max(5,min(2000,min(max(1,self.width()-70)/(max(xs)-min(xs)),max(1,self.height()-80)/(max(ys)-min(ys)))))
        self.update()

    def world_to_screen(self, x, y):
        """@brief 将地图米制坐标x、y转换为屏幕像素，地图正Y在屏幕向上。"""
        return QPointF(self.width()/2+(x-self.center.x())*self.scale,self.height()/2-(y-self.center.y())*self.scale)

    def screen_to_world(self, point):
        """@brief 将屏幕point逆变换为地图坐标，用于选点、平移和缩放。"""
        return QPointF(self.center.x()+(point.x()-self.width()/2)/self.scale,self.center.y()-(point.y()-self.height()/2)/self.scale)

    def zoom(self, factor, anchor=None):
        """@brief 按factor缩放并保持anchor像素对应地图位置不动。"""
        anchor=anchor or QPointF(self.width()/2,self.height()/2)
        before=self.screen_to_world(anchor)
        self.scale=max(5,min(2000,self.scale*factor))
        after=self.screen_to_world(anchor)
        self.center+=before-after;self.update()

    def wheelEvent(self, event):
        """@brief 在Windows本地处理滚轮缩放，不依赖VNC转发滚轮事件。"""
        self.zoom(1.2 if event.angleDelta().y()>0 else 1/1.2,event.position());event.accept()

    def mousePressEvent(self, event):
        """@brief 右键或平移模式拖动画布，选点模式左键记录目标位置。"""
        if event.button() in (Qt.RightButton,Qt.MiddleButton) or (event.button()==Qt.LeftButton and self.mode=='pan'):
            self.pan_start=event.position();return
        if event.button()==Qt.LeftButton and self.connected and self.map_geometry:
            self.drag_start=self.screen_to_world(event.position());self.drag_end=event.position();self.update()

    def mouseMoveEvent(self, event):
        """@brief 更新平移或方向箭头预览，拖动期间不发送导航命令。"""
        if self.pan_start is not None:
            delta=event.position()-self.pan_start
            self.center+=QPointF(-delta.x()/self.scale,delta.y()/self.scale)
            self.pan_start=event.position();self.update()
        elif self.drag_start is not None:
            self.drag_end=event.position();self.update()

    def mouseReleaseEvent(self, event):
        """@brief 松开左键后校验空闲栅格并提交位置朝向，过短拖动不提交。"""
        if self.pan_start is not None:
            self.pan_start=None;return
        if event.button()!=Qt.LeftButton or self.drag_start is None:return
        start=self.drag_start;mode=self.mode;end=self.screen_to_world(event.position())
        self.drag_start=self.drag_end=None
        if math.hypot(end.x()-start.x(),end.y()-start.y())*self.scale<8:
            self.notice.emit('请按住左键拖出车头方向，至少8像素后松开');self.update();return
        if not self.connected or not self.map_geometry or self.map_geometry.frame!='map':
            self.notice.emit('尚未连接或地图坐标不是map，选点未发送');return
        if not self.map_geometry.free(self.cells,start.x(),start.y()):
            self.notice.emit('请选择地图中的已知空闲区域，不能选墙面、未知区或地图外');self.update();return
        yaw=math.atan2(end.y()-start.y(),end.x()-start.x())
        self.selected=(start.x(),start.y(),yaw,mode)
        self.set_mode('pan')
        self.posePicked.emit(mode,start.x(),start.y(),yaw)

    def draw_arrow(self, painter, pose, color, label, length=.22):
        """@brief 在pose位置绘制朝向箭头，color和label区分实车与人工选点。"""
        x,y,yaw=pose;start=self.world_to_screen(x,y)
        tip=self.world_to_screen(x+length*math.cos(yaw),y+length*math.sin(yaw))
        painter.setPen(QPen(QColor(color),3));painter.drawLine(start,tip)
        for offset in (-.55,.55):
            tail=self.world_to_screen(x+length*math.cos(yaw)-.07*math.cos(yaw+offset),y+length*math.sin(yaw)-.07*math.sin(yaw+offset))
            painter.drawLine(tip,tail)
        painter.drawText(start+QPointF(8,-12),label)

    def paintEvent(self, event):
        """@brief 在Windows绘制地图、真实激光、车体及路径，无坐标时不虚构叠加。"""
        painter=QPainter(self);painter.fillRect(self.rect(),QColor('#111c2b'))
        painter.setRenderHint(QPainter.Antialiasing)
        if not self.map_geometry:
            painter.setPen(QColor('#9badc3'));painter.drawText(self.rect(),Qt.AlignCenter,'等待地图 · 请启动建图或地图定位导航');return
        g=self.map_geometry;origin=self.world_to_screen(g.ox,g.oy)
        painter.save();painter.translate(origin);painter.rotate(-math.degrees(g.yaw))
        painter.scale(self.scale*g.resolution,-self.scale*g.resolution)
        painter.drawImage(QRectF(0,0,g.width,g.height),self.image);painter.restore()
        fresh=self.connected and time.monotonic()-self.scene_received<.8
        scene=self.scene if fresh else {}
        if scene.get('frame')==g.frame:
            painter.setPen(QPen(QColor('#41dc91'),3))
            path=scene.get('path',[])
            if len(path)>1:painter.drawPolyline(QPolygonF([self.world_to_screen(x,y) for x,y in path]))
            painter.setPen(QPen(QColor('#ff625c'),3))
            for x,y in scene.get('laser',[]):painter.drawPoint(self.world_to_screen(x,y))
            pose=scene.get('pose')
            if pose:
                polygon=transform_points([[.125,.10],[.125,-.10],[-.125,-.10],[-.125,.10]],pose)
                painter.setPen(QPen(QColor('#57b7ff'),2));painter.setBrush(QColor(87,183,255,60))
                painter.drawPolygon(QPolygonF([self.world_to_screen(x,y) for x,y in polygon]));painter.setBrush(Qt.NoBrush)
                self.draw_arrow(painter,pose,'#57b7ff','车头')
        if self.selected:
            self.draw_arrow(painter,self.selected[:3],'#ffc766','选定初始位置' if self.selected[3]=='initial' else '选定目标')
        if self.drag_start is not None and self.drag_end is not None:
            end=self.screen_to_world(self.drag_end);start=self.drag_start
            yaw=math.atan2(end.y()-start.y(),end.x()-start.x())
            self.draw_arrow(painter,(start.x(),start.y(),yaw),'#ffc766','松开提交',max(.08,math.hypot(end.x()-start.x(),end.y()-start.y())))
        painter.setPen(QColor('#dce7f5'))
        status='Windows本地绘制 · 左键平移 / 滚轮缩放 · 红色激光 / 蓝色车体 / 绿色路径'
        painter.drawText(12,22,status)
        if not fresh:
            painter.setPen(QColor('#ffbd69'));painter.drawText(12,44,'实时叠加未连接或已超时；保留的地图仅供查看')
        elif scene.get('reason'):
            painter.setPen(QColor('#ffbd69'));painter.drawText(12,44,scene['reason'])
