"""验证Windows地图坐标、实际鼠标拖动与位置请求校验，不连接硬件。"""
import importlib
import math
import os
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

os.environ['QT_QPA_PLATFORM']='offscreen'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6.QtCore import QPoint,Qt,QPointF
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication
from desktop.navigation_view import NavigationView
from shared.navigation_geometry import MapGeometry,pose_values


def map_data():
    """@brief 提供带负原点的测试栅格，用于验证坐标而不是展示模拟硬件状态。"""
    return {'width':12,'height':10,'resolution':.05,'origin':[-.3,-.2,0.0],'frame':'map','data':[0]*120}


class GeometryTests(unittest.TestCase):
    def test_rotated_origin_roundtrip(self):
        """@brief 非零原点及90度旋转下，栅格中心正反变换必须一致。"""
        data=map_data();data['origin']=[1.,2.,math.pi/2];g=MapGeometry(data)
        for x,y in [(0,0),(3,5),(11,9)]:
            wx,wy=g.cell_to_world(x+.5,y+.5)
            self.assertEqual(g.world_to_cell(wx,wy),(x,y))

    def test_unknown_wall_and_invalid_pose(self):
        """@brief 未知区、占据区、地图外和非有限输入不能作为目标。"""
        data=map_data();g=MapGeometry(data);data['data'][0]=-1;data['data'][1]=100
        for cell in [(0,0),(1,0)]:self.assertFalse(g.free(data['data'],*g.cell_to_world(cell[0]+.5,cell[1]+.5)))
        self.assertFalse(g.free(data['data'],100,100))
        for bad in [float('nan'),float('inf'),True,'1']:
            with self.assertRaises(ValueError):pose_values({'x':bad,'y':0.,'yaw':0.})


class ViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 创建离线Qt事件环境，真实处理鼠标事件。"""
        cls.app=QApplication.instance() or QApplication([])

    def setUp(self):
        """@brief 创建本地画布和测试地图，不连接树莓派。"""
        self.view=NavigationView();self.view.resize(800,600)
        self.view.set_map(map_data());self.view.fit_map();self.view.set_connected(True)
        self.view.show();self.app.processEvents();self.sent=[]
        self.view.posePicked.connect(lambda *args:self.sent.append(args))
        self.addCleanup(self.view.close)

    def test_screen_roundtrip_and_zoom_anchor(self):
        """@brief 验证鼠标定位的逆变换和围绕光标缩放的稳定性。"""
        point=self.view.world_to_screen(.123,-.034)
        world=self.view.screen_to_world(point)
        self.assertAlmostEqual(world.x(),.123);self.assertAlmostEqual(world.y(),-.034)
        anchor=QPointF(150,200);before=self.view.screen_to_world(anchor)
        self.view.zoom(1.5,anchor);after=self.view.screen_to_world(anchor)
        self.assertAlmostEqual(before.x(),after.x());self.assertAlmostEqual(before.y(),after.y())

    def test_drag_sends_only_after_release(self):
        """@brief 真实拖动在松开时只发送一次，并正确转换角度。"""
        self.view.set_mode('initial')
        start=self.view.world_to_screen(0.,0.).toPoint();end=start+QPoint(40,-30)
        QTest.mousePress(self.view,Qt.LeftButton,pos=start)
        QTest.mouseMove(self.view,end);self.assertEqual(self.sent,[])
        QTest.mouseRelease(self.view,Qt.LeftButton,pos=end)
        self.assertEqual(len(self.sent),1)
        mode,x,y,yaw=self.sent[0]
        self.assertEqual(mode,'initial');self.assertAlmostEqual(x,0.,delta=.001)
        self.assertAlmostEqual(y,0.,delta=.001);self.assertAlmostEqual(yaw,math.atan2(30,40))

    def test_wall_disconnected_and_pan_do_not_send(self):
        """@brief 墙面选点、离线点击和平移均不能触发目标消息。"""
        data=map_data();data['data']=[100]*120;self.view.set_map(data);self.view.set_mode('goal')
        start=self.view.world_to_screen(0.,0.).toPoint();end=start+QPoint(40,0)
        QTest.mousePress(self.view,Qt.LeftButton,pos=start);QTest.mouseRelease(self.view,Qt.LeftButton,pos=end)
        self.assertEqual(self.sent,[])
        self.view.set_connected(False);self.view.set_mode('goal')
        QTest.mousePress(self.view,Qt.LeftButton,pos=start);QTest.mouseRelease(self.view,Qt.LeftButton,pos=end)
        self.assertEqual(self.sent,[])

    def test_render_preserves_grid_y_and_origin_rotation(self):
        """@brief 检查旋转地图中各栅格中心像素，避免图像翻转导致目标落错位置。"""
        data={'frame':'map','width':2,'height':2,'resolution':.5,'origin':[1.,2.,math.pi/2],'data':[0,100,-1,0]}
        self.view.set_map(data);self.view.fit_map();self.app.processEvents();image=self.view.grab().toImage()
        for x,y,expected in [(0,0,255),(1,0,0),(0,1,110),(1,1,255)]:
            world=self.view.map_geometry.cell_to_world(x+.5,y+.5)
            point=self.view.world_to_screen(*world).toPoint()
            self.assertAlmostEqual(image.pixelColor(point).red(),expected,delta=1)

    def test_stale_scene_hides_vehicle(self):
        """@brief 网络延迟超过有效期时隐藏旧车体，不能把历史位置显示为实时定位。"""
        scene={'frame':'map','pose':[0.,0.,0.],'laser':[],'path':[],'reason':''}
        self.view.set_scene(scene);point=self.view.world_to_screen(-.05,.04).toPoint()
        before=self.view.grab().toImage().pixelColor(point)
        self.assertGreater(before.blue(),before.red())
        self.view.set_scene(scene,age=2.)
        after=self.view.grab().toImage().pixelColor(point)
        self.assertEqual(after.red(),255);self.assertEqual(after.green(),255)


class GatewayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """@brief 加载真实网关位置处理函数，只替代平台专属文件锁。"""
        with patch.dict(sys.modules,{'fcntl':types.ModuleType('fcntl')}):cls.module=importlib.import_module('pi.gateway')

    def test_valid_initial_pose_and_expired_wall_rejection(self):
        """@brief 验证请求期限及地图空闲区检查先于调用ROS接口。"""
        gateway=self.module.Gateway.__new__(self.module.Gateway)
        gateway.current_map=map_data();calls=[];events=[]
        gateway.navigation=types.SimpleNamespace(set_initial_pose=calls.append,submit_goal=calls.append,result='提交')
        command={'action':'navigation_initial_pose','x':0.,'y':0.,'yaw':.3,'deadline':time.time()+.8}
        with patch.object(self.module,'emit',side_effect=lambda *a,**kw:events.append(kw)):
            gateway.map_pose_command(command);self.assertEqual(len(calls),1);self.assertTrue(events[-1]['success'])
            gateway.map_pose_command({**command,'deadline':time.time()-1});self.assertEqual(len(calls),1)
            gateway.current_map['data']=[100]*120
            gateway.map_pose_command(command);self.assertEqual(len(calls),1);self.assertFalse(events[-1]['success'])


if __name__=='__main__':unittest.main()
