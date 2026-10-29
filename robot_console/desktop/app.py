"""小车调试上位机：连接、程序管理、数据监控和按住式遥控。"""
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time

from PySide6.QtCore import Qt, QTimer, QSettings, QProcess
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QGridLayout, QLabel, QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox,
    QComboBox, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView, QPlainTextEdit,
    QFileDialog, QMessageBox, QListWidget, QStackedWidget, QSplitter, QButtonGroup)

from desktop.connection import Connection
from desktop.widgets import SpeedPlot, Attitude, SpatialView, COLORS
from desktop.navigation_view import NavigationView

STYLE = """
QMainWindow,QWidget {background:#0c1522;color:#dce7f5;font-family:'Microsoft YaHei UI';font-size:13px;}
QGroupBox {background:#142133;border:1px solid #26394e;border-radius:10px;margin-top:14px;padding:15px 10px 10px;font-weight:bold;}
QGroupBox::title {subcontrol-origin:margin;left:12px;padding:0 5px;color:#9fb4cd;}
QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox {background:#0d1928;border:1px solid #344960;border-radius:5px;padding:7px;min-height:22px;}
QPushButton {background:#233c55;border:1px solid #345773;border-radius:6px;padding:9px 16px;}
QPushButton:hover {background:#315571;}
QPushButton:pressed {background:#367969;}
QPushButton:disabled {background:#172333;color:#627287;border-color:#263346;}
QPushButton#primary {background:#147e72;color:white;}
QPushButton#stop {background:#a83e48;color:white;font-weight:bold;}
QListWidget {background:#101d2d;border:0;font-size:15px;padding:8px;outline:0;}
QListWidget::item {padding:16px 12px;margin:3px 0;border-radius:6px;}
QListWidget::item:selected {background:#24445a;color:#62e2cf;}
QTableWidget {background:#111c2b;alternate-background-color:#172638;gridline-color:#283a50;border:0;}
QHeaderView::section {background:#203247;padding:10px;border:0;}
QPlainTextEdit {background:#0a121e;border:1px solid #26394e;font-family:Consolas,'Microsoft YaHei UI';}
QLabel#muted {color:#8fa5bf;}
QStatusBar {background:#101d2d;color:#9fb4cd;}
"""


def group(title, layout):
    """@brief 将布局放入统一样式的功能卡片。"""
    box=QGroupBox(title)
    box.setLayout(layout)
    return box


def button(text, callback, name=''):
    """@brief 创建按钮并绑定固定操作，name 用于强调外观。"""
    item=QPushButton(text)
    item.setObjectName(name)
    item.clicked.connect(callback)
    return item


class Window(QMainWindow):
    def __init__(self):
        """@brief 构建上位机窗口，初始所有硬件数据均为未接入。"""
        super().__init__()
        self.setWindowTitle('小车调试台 | ROS 2 · STM32 · K230')
        self.resize(1420,920)
        self.setMinimumSize(1120,740)
        self.worker=None
        self.closing=False
        self.stamps={}
        self.server_armed=False
        self.wheel_control=False
        self.servo_allowed=False
        self.servo_capable=False
        self.navigation_mode=False
        self.navigation_ready=False
        self.yaw_loaded=False
        self.yaw_pending=False
        self.direction=(0.,0.)
        self.record_file=None
        self.recorder=None
        self.settings=QSettings('RobotWorkbench','Console')
        installed_viewer=Path('C:/Program Files/RealVNC/VNC Viewer/vncviewer.exe')
        if not self.settings.value('vnc_viewer','') and installed_viewer.is_file():
            self.settings.setValue('vnc_viewer',str(installed_viewer))
        central=QWidget()
        outer=QVBoxLayout(central)
        outer.setContentsMargins(18,14,18,10)
        title=QLabel('小车调试台')
        title.setStyleSheet('font-size:25px;font-weight:bold;color:#f0f6ff;')
        subtitle=QLabel('ROS 2 / STM32 / K230     实时监控与程序管理')
        subtitle.setObjectName('muted')
        outer.addWidget(title)
        outer.addWidget(subtitle)
        outer.addWidget(self.connection_panel())
        body=QHBoxLayout()
        self.nav=QListWidget()
        self.nav.setFixedWidth(165)
        self.nav.addItems(['总览与遥控','四轮速度','IMU 姿态','计算结果','地图与导航','程序与日志'])
        self.pages=QStackedWidget()
        for page in (self.overview_page(),self.wheels_page(),self.imu_page(),self.results_page(),
                     self.map_page(),self.program_page()):
            self.pages.addWidget(page)
        self.nav.currentRowChanged.connect(self.pages.setCurrentIndex)
        self.nav.setCurrentRow(0)
        body.addWidget(self.nav)
        body.addWidget(self.pages,1)
        outer.addLayout(body,1)
        footer=QHBoxLayout()
        self.record_btn=button('开始 CSV 记录',self.toggle_record)
        self.record_status=QLabel('未记录 · 无数据时不生成模拟采样')
        self.record_status.setObjectName('muted')
        footer.addWidget(self.record_btn)
        footer.addWidget(self.record_status,1)
        footer.addWidget(button('禁能 / 停止遥控',self.disarm,'stop'))
        outer.addLayout(footer)
        self.setCentralWidget(central)
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        QApplication.instance().applicationStateChanged.connect(self.application_state)
        self.statusBar().showMessage('地图由Windows本地绘制 · 定位、规划与运动控制由树莓派执行')

    def connection_panel(self):
        """@brief 连接栏仅保存地址和用户名，密码不写入设置。"""
        row=QHBoxLayout()
        self.host=QLineEdit(self.settings.value('host','192.168.10.193'))
        self.host.setMaximumWidth(180)
        self.port=QSpinBox()
        self.port.setRange(1,65535)
        self.port.setValue(int(self.settings.value('port',22)))
        self.port.setMaximumWidth(90)
        self.user=QLineEdit(self.settings.value('user','xu'))
        self.user.setMaximumWidth(110)
        self.password=QLineEdit()
        self.password.setEchoMode(QLineEdit.Password)
        self.password.setPlaceholderText('SSH / sudo 密码')
        self.password.setMaximumWidth(190)
        self.connect_btn=button('连接树莓派',self.toggle_connection,'primary')
        self.connection_status=QLabel('未连接')
        for label,widget in [('地址',self.host),('端口',self.port),('用户',self.user),('密码',self.password)]:
            row.addWidget(QLabel(label))
            row.addWidget(widget)
        row.addWidget(self.connect_btn)
        row.addWidget(self.connection_status,1)
        return group('SSH 加密连接',row)

    def overview_page(self):
        """@brief 展示各数据源健康状态与按住式遥控面板。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        cards=QGridLayout()
        self.health={}
        for i,(key,title) in enumerate([('system','树莓派'),('wheels','STM32 底盘'),('imu','IMU'),('k230','K230')]):
            value=QLabel('未接入')
            value.setStyleSheet('font-size:22px;color:#8fa5bf;')
            info=QVBoxLayout()
            info.addWidget(value)
            cards.addWidget(group(title,info),0,i)
            self.health[key]=value
        layout.addLayout(cards)
        lower=QHBoxLayout()
        controls=QVBoxLayout()
        serial=QHBoxLayout()
        self.serial_ports=QComboBox()
        self.serial_ports.setMinimumWidth(180)
        self.serial_ports.setPlaceholderText('选择底盘 UART3 或 USB TTL')
        serial.addWidget(self.serial_ports,1)
        serial.addWidget(button('刷新',lambda:self.send('ports')))
        controls.addLayout(serial)
        serial_actions=QHBoxLayout()
        serial_actions.addWidget(button('打开底盘串口',self.open_serial))
        serial_actions.addWidget(button('关闭串口',lambda:self.send('serial_close')))
        controls.addLayout(serial_actions)
        self.arm_btn=button('使能遥控',lambda:self.send('arm'),'primary')
        self.arm_btn.setEnabled(False)
        controls.addWidget(self.arm_btn)
        self.motion_status=QLabel('等待新版固件反馈；默认禁能')
        self.motion_status.setWordWrap(True)
        controls.addWidget(self.motion_status)
        limits=QHBoxLayout()
        self.vx=QDoubleSpinBox()
        self.vx.setRange(.02,.2)
        self.vx.setSingleStep(.02)
        self.vx.setValue(.1)
        self.vx.setSuffix(' m/s')
        self.wz=QDoubleSpinBox()
        self.wz.setRange(.1,.7)
        self.wz.setSingleStep(.1)
        self.wz.setValue(.4)
        self.wz.setSuffix(' rad/s')
        limits.addWidget(QLabel('线速度'))
        limits.addWidget(self.vx)
        limits.addWidget(QLabel('角速度'))
        limits.addWidget(self.wz)
        controls.addLayout(limits)
        arrows=QGridLayout()
        self.arrow_buttons=[]
        for text,row,col,direction in [('前进',0,1,(1,0)),('左转',1,0,(0,1)),
                                       ('后退',2,1,(-1,0)),('右转',1,2,(0,-1))]:
            b=QPushButton(text+'\n按住运行')
            b.setMinimumSize(100,65)
            b.setAutoRepeat(False)
            b.setEnabled(False)
            b.pressed.connect(lambda d=direction:self.set_direction(d))
            b.released.connect(lambda:self.set_direction((0,0)))
            arrows.addWidget(b,row,col)
            self.arrow_buttons.append(b)
        arrows.addWidget(button('停止',lambda:self.set_direction((0,0)),'stop'),1,1)
        controls.addLayout(arrows)
        note=QLabel('仅按住按钮时发送非零速度；松开、切页、窗口失焦或失联即撤销。\n重新连接后需再次使能。软件停止不替代物理急停。')
        note.setObjectName('muted')
        note.setWordWrap(True)
        controls.addWidget(note)
        lower.addWidget(group('底盘遥控',controls),1)
        right=QVBoxLayout()
        servo_layout=QVBoxLayout()
        servo_row=QHBoxLayout()
        self.servo_pulse=QSpinBox()
        self.servo_pulse.setRange(1000,2000)
        self.servo_pulse.setValue(1500)
        self.servo_pulse.setSuffix(' μs')
        servo_row.addWidget(self.servo_pulse)
        self.servo_apply=button('输出目标脉宽',lambda:self.send('servo',pulse_us=self.servo_pulse.value()))
        self.servo_release=button('关闭舵机输出',lambda:self.send('servo',pulse_us=0))
        self.servo_apply.setEnabled(False)
        self.servo_release.setEnabled(False)
        servo_row.addWidget(self.servo_apply)
        servo_layout.addLayout(servo_row)
        servo_layout.addWidget(self.servo_release)
        self.servo_status=QLabel('未接入；确认舵机型号和限位后才能开启。')
        self.servo_status.setWordWrap(True)
        servo_layout.addWidget(self.servo_status)
        servo_note=QLabel('失联保持最后输出；反馈为PWM指令值，并非实际角度。')
        servo_note.setWordWrap(True)
        servo_layout.addWidget(servo_note)
        right.addWidget(group('前部舵机',servo_layout))
        self.system_info=QLabel('等待树莓派系统状态')
        self.system_info.setWordWrap(True)
        right.addWidget(self.system_info)
        right.addWidget(QLabel('接入顺序：\n1. 连接树莓派\n2. 选择 /dev/ttyAMA3（PC10/PC11）\n3. 收到 wheels-v2 固件反馈\n4. 人工使能，按住方向按钮'))
        self.overview_log=QPlainTextEdit()
        self.overview_log.setReadOnly(True)
        self.overview_log.setMaximumBlockCount(150)
        right.addWidget(self.overview_log,1)
        lower.addWidget(group('系统状态与事件',right),1)
        layout.addLayout(lower,1)
        self.pages.currentChanged.connect(lambda _:self.disarm())
        return page

    def wheels_page(self):
        """@brief 为四轮提供目标、实测、误差和有界实时曲线。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        layout.addWidget(QLabel('轮序：A/B 一侧，C/D 另一侧；左转为 AB 后退、CD 前进。'))
        self.wheel_table=QTableWidget(4,5)
        self.wheel_table.setHorizontalHeaderLabels(['车轮','目标 m/s','实测 m/s','误差 m/s','累计计数（32位）'])
        self.wheel_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.wheel_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.wheel_table.setMaximumHeight(220)
        for row,name in enumerate('ABCD'):
            for col in range(5):
                self.wheel_table.setItem(row,col,QTableWidgetItem(name if col==0 else '—'))
        layout.addWidget(self.wheel_table)
        legend=QHBoxLayout()
        for name,color in zip('ABCD',COLORS):
            item=QLabel(name+'  实测')
            item.setStyleSheet('color:'+color)
            legend.addWidget(item)
        layout.addLayout(legend)
        self.speed_plot=SpeedPlot()
        layout.addWidget(self.speed_plot,1)
        self.wheel_age=QLabel('等待底盘数据')
        self.wheel_age.setWordWrap(True)
        layout.addWidget(self.wheel_age)
        return page

    def imu_page(self):
        """@brief 区分原始惯性量与有效解算姿态，允许后续选择独立 I2C IMU。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        row=QHBoxLayout()
        row.addWidget(QLabel('显示数据源（控制来源由树莓派参数选择）'))
        self.imu_source=QComboBox()
        self.imu_source.addItems(['STM32 板载 IMU','/imu/data','/imu/data_raw'])
        self.imu_source.currentIndexChanged.connect(self.clear_imu)
        row.addWidget(self.imu_source)
        row.addStretch()
        layout.addLayout(row)
        tuning=QVBoxLayout()
        fields=QGridLayout()
        self.yaw_fields={}
        for col,(key,title,high,step,initial) in enumerate((
            ('kp','比例 Kp',.2,.005,.010),('ki','积分 Ki',.02,.0005,0.),
            ('kd','微分 Kd',.05,.001,.002),('max_wz','纠偏限幅 rad/s',.3,.01,.08))):
            fields.addWidget(QLabel(title),0,col)
            editor=QDoubleSpinBox()
            editor.setDecimals(4)
            editor.setRange(0.,high)
            editor.setSingleStep(step)
            editor.setValue(initial)
            editor.setEnabled(False)
            editor.valueChanged.connect(lambda _:self.yaw_status.setText('输入已修改，尚未应用到小车'))
            fields.addWidget(editor,1,col)
            self.yaw_fields[key]=editor
        tuning.addLayout(fields)
        actions=QHBoxLayout()
        self.yaw_read=button('读取当前参数',lambda:self.request_yaw())
        self.yaw_apply=button('应用本次',lambda:self.request_yaw(False))
        self.yaw_save=button('应用并保存',lambda:self.request_yaw(True),'primary')
        for b in (self.yaw_read,self.yaw_apply,self.yaw_save):
            b.setEnabled(False)
            actions.addWidget(b)
        tuning.addLayout(actions)
        self.yaw_current=QLabel('当前生效参数：尚未读取')
        self.yaw_status=QLabel('连接后读取参数；先禁能小车，再应用修改。')
        self.yaw_status.setWordWrap(True)
        tuning.addWidget(self.yaw_current)
        tuning.addWidget(self.yaw_status)
        layout.addWidget(group('直行航向调参',tuning))
        self.attitude=Attitude()
        layout.addWidget(self.attitude,1)
        self.imu_values=QLabel('Roll —     Pitch —     Yaw —')
        self.imu_values.setStyleSheet('font-size:22px;')
        self.imu_detail=QLabel('等待 IMU 数据；未提供有效姿态时不显示角度。')
        self.imu_detail.setWordWrap(True)
        layout.addWidget(self.imu_values)
        layout.addWidget(self.imu_detail)
        return page

    def results_page(self):
        """@brief 提供 K230 检测表与树莓派任意 JSON 计算结果面板。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        self.k230_info=QLabel('K230 未接入 · 串口协议预留 · 尚无 YOLO 模型数据')
        layout.addWidget(self.k230_info)
        self.detections=QTableWidget(0,5)
        self.detections.setHorizontalHeaderLabels(['类别 ID','类别名称','中心 X / px','中心 Y / px','置信度'])
        self.detections.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.detections.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.detections,1)
        layout.addWidget(QLabel('树莓派计算结果 · /pi/results_json'))
        self.pi_results=QPlainTextEdit()
        self.pi_results.setReadOnly(True)
        self.pi_results.setPlaceholderText('等待 JSON 结果，例如定位质量、算法耗时或任务状态')
        layout.addWidget(self.pi_results,1)
        return page

    def map_page(self):
        """@brief 在Windows本地绘制导航地图，远程RViz仅作为可选调试入口。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        row=QHBoxLayout()
        row.addWidget(button('启动雷达',lambda:self.send('start',name='lidar')))
        row.addWidget(button('启动纯雷达建图',lambda:self.send('start',name='mapping')))
        row.addWidget(button('停止建图',lambda:self.send('stop',name='mapping')))
        row.addWidget(button('保存地图到树莓派',lambda:self.send('save_map')))
        row.addStretch()
        layout.addLayout(row)
        rviz_box=QGroupBox('远程 RViz / VNC（可选，会增加树莓派负载）')
        rviz_box.setCheckable(True);rviz_box.setChecked(False)
        rviz_outer=QVBoxLayout(rviz_box);rviz_content=QWidget();rviz=QHBoxLayout(rviz_content)
        rviz.addWidget(button('启动 RViz',lambda:self.send('start',name='rviz'),'primary'))
        rviz.addWidget(button('停止 RViz',lambda:self.send('stop',name='rviz')))
        rviz.addWidget(button('通过 VNC 查看 RViz',self.open_vnc))
        self.vnc_kind=QComboBox()
        self.vnc_kind.addItems(['TigerVNC','RealVNC'])
        if 'realvnc' in self.settings.value('vnc_viewer','').lower():
            self.vnc_kind.setCurrentIndex(1)
        rviz.addWidget(self.vnc_kind)
        rviz.addWidget(button('选择 VNC 软件',self.choose_vnc))
        rviz.addWidget(button('打开导航 RViz',lambda:self.send('start',name='navigation_rviz')))
        rviz.addWidget(button('停止导航 RViz',lambda:self.send('stop',name='navigation_rviz')))
        rviz_outer.addWidget(rviz_content);rviz_content.setVisible(False)
        rviz_box.toggled.connect(rviz_content.setVisible)
        navigation_row=QHBoxLayout()
        navigation_row.addWidget(button('启动地图定位导航',lambda:self.send('start',name='navigation')))
        navigation_row.addWidget(button('停止导航程序',lambda:self.send('stop',name='navigation')))
        self.navigation_allow=button('允许自主控制',lambda:self.send('navigation_enable'),'primary')
        self.navigation_allow.setEnabled(False)
        navigation_row.addWidget(self.navigation_allow)
        navigation_row.addWidget(button('取消导航并停车',lambda:self.send('navigation_cancel'),'stop'))
        layout.addLayout(navigation_row)
        self.navigation_status=QLabel('停止建图→打开底盘串口→启动地图定位导航→在下方设置初始位置；不需要打开RViz。')
        self.navigation_status.setWordWrap(True)
        layout.addWidget(self.navigation_status)
        views=QSplitter()
        self.scan_view=SpatialView()
        self.map_view=NavigationView()
        self.map_view.posePicked.connect(self.map_pose_picked)
        self.map_view.notice.connect(self.map_notice)
        tools=QHBoxLayout();self.map_mode_buttons={};self.map_modes=QButtonGroup(self)
        for mode,title in [('pan','平移地图'),('initial','设置初始位置'),('goal','设置导航目标')]:
            tool=QPushButton(title);tool.setCheckable(True)
            tool.clicked.connect(lambda checked=False,m=mode:self.map_view.set_mode(m))
            self.map_modes.addButton(tool);self.map_mode_buttons[mode]=tool;tools.addWidget(tool)
        self.map_mode_buttons['pan'].setChecked(True)
        self.map_view.modeChanged.connect(lambda mode:self.map_mode_buttons[mode].setChecked(True))
        tools.addWidget(button('适配地图',self.map_view.fit_map))
        tools.addWidget(button('放大 +',lambda:self.map_view.zoom(1.25)))
        tools.addWidget(button('缩小 −',lambda:self.map_view.zoom(.8)))
        tools.addWidget(button('刷新地图',lambda:self.send('map_refresh')))
        raw_scan=QPushButton('原始激光预览');raw_scan.setCheckable(True)
        raw_scan.toggled.connect(self.scan_view.setVisible);tools.addWidget(raw_scan)
        layout.addLayout(tools)
        self.map_instruction=QLabel('选择模式后，在地图上按住左键拖出车头方向；松开才提交。右键拖动平移，滚轮或按钮缩放。')
        self.map_instruction.setWordWrap(True);layout.addWidget(self.map_instruction)
        views.addWidget(self.map_view)
        views.addWidget(self.scan_view);self.scan_view.setVisible(False)
        layout.addWidget(views,1)
        self.spatial_status=QLabel('Windows本地渲染 · 等待地图与真实定位数据')
        layout.addWidget(self.spatial_status)
        layout.addWidget(rviz_box)
        return page

    def map_notice(self, text):
        """@brief 显示地图操作结果，明确区分人工选点与导航执行成功。"""
        self.map_instruction.setText(text);self.add_log(text)

    def map_pose_picked(self, mode, x, y, yaw):
        """@brief 发送地图中的米制位置和弧度朝向，目标仍由树莓派检查并执行。"""
        if mode=='goal' and (not self.navigation_mode or not self.navigation_ready):
            self.map_notice('目标未发送：请先等待定位就绪，再点击允许自主控制');return
        if mode=='initial' and (self.server_armed or self.navigation_mode):
            self.map_notice('初始位置未发送：请先取消导航并停车');return
        action='navigation_initial_pose' if mode=='initial' else 'navigation_goal'
        self.map_notice(f'已提交选点：x={x:.2f}m，y={y:.2f}m，朝向={yaw*180/3.141592653589793:.1f}°；等待树莓派确认')
        self.send(action,x=x,y=y,yaw=yaw)

    def program_page(self):
        """@brief 管理预设程序并显示有上限的日志，禁用尚未接入的功能。"""
        page=QWidget()
        layout=QVBoxLayout(page)
        self.program_labels={}
        for key,title in [('lidar','DELTA2C 雷达'),('mapping','纯雷达建图测试'),('rviz','RViz 可视化'),('navigation','地图定位与导航'),('navigation_rviz','导航 RViz')]:
            row=QHBoxLayout()
            row.addWidget(QLabel(title),1)
            state=QLabel('未启动')
            self.program_labels[key]=state
            row.addWidget(state)
            row.addWidget(button('启动',lambda checked=False,k=key:self.send('start',name=k)))
            row.addWidget(button('停止',lambda checked=False,k=key:self.send('stop',name=k)))
            layout.addLayout(row)
        layout.addWidget(QLabel('底盘串口由总览页管理；导航显示在Windows绘制，通常无需启动远程RViz。'))
        layout.addWidget(QLabel('断开上位机时，网关禁能底盘并停止本会话启动的程序；不接管其他终端的程序。'))
        self.log=QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        layout.addWidget(self.log,1)
        layout.addWidget(button('导出当前日志',self.export_log))
        return page

    def toggle_connection(self):
        """@brief 启动后台连接或请求断开，连接结束前禁止重复创建线程。"""
        if self.worker:
            self.disarm()
            self.worker.stop()
            self.connect_btn.setEnabled(False)
            self.connection_status.setText('正在断开…')
            return
        if not self.host.text().strip() or not self.user.text().strip():
            QMessageBox.information(self,'连接参数','请填写树莓派地址和用户名。')
            return
        self.settings.setValue('host',self.host.text().strip())
        self.settings.setValue('port',self.port.value())
        self.settings.setValue('user',self.user.text().strip())
        self.worker=Connection(self.host.text().strip(),self.port.value(),self.user.text().strip(),self.password.text())
        self.password.clear()
        self.worker.message.connect(self.on_message)
        self.worker.status.connect(self.connection_status.setText)
        self.worker.failure.connect(self.connection_error)
        self.worker.finished.connect(self.connection_finished)
        self.connect_btn.setText('断开连接')
        self.worker.start()

    def connection_error(self,text):
        """@brief 将连接失败显示在日志及明确的错误窗口中。"""
        self.add_log('连接失败：'+text)
        QMessageBox.warning(self,'连接失败',text)

    def connection_finished(self):
        """@brief 断开后清除控制状态，历史数据保留但显示超时。"""
        self.worker.deleteLater()
        self.worker=None
        self.server_armed=False
        self.direction=(0.,0.)
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText('连接树莓派')
        if self.closing:
            self.close()

    def send(self,action,**values):
        """@brief 只有握手成功后才向后台发送界面请求。"""
        if self.worker and self.worker.ready:
            self.worker.send({'action':action,**values})
        elif action not in ('motion','disarm'):
            self.add_log('操作未执行：请先连接树莓派并等待网关就绪')

    def open_serial(self):
        """@brief 使用树莓派报告的 UART3 或 USB 设备路径打开底盘通信。"""
        if self.serial_ports.currentText():
            self.send('serial_open',port=self.serial_ports.currentText())
        else:
            self.add_log('没有可选底盘串口，请确认 UART3 已启用或 USB TTL 已连接后刷新')

    def choose_vnc(self):
        """@brief 选择本机已有的 VNC Viewer，路径保存在本机设置中。"""
        path,_=QFileDialog.getOpenFileName(self,'选择 TigerVNC 或 RealVNC 的 Viewer 程序','','程序 (*.exe)')
        if path:
            self.settings.setValue('vnc_viewer',path)

    def open_vnc(self):
        """@brief 通过 SSH 内的本机临时端口打开 VNC 查看树莓派桌面。"""
        if not self.navigation_mode:
            self.disarm()
        if not self.worker or not self.worker.ready or self.worker.vnc_port is None:
            self.add_log('请先连接树莓派并等待 VNC 隧道就绪')
            return
        path=self.settings.value('vnc_viewer','')
        if not path or not Path(path).is_file():
            self.choose_vnc()
            path=self.settings.value('vnc_viewer','')
        if not path:
            return
        port=self.worker.vnc_port
        address=f'127.0.0.1::{port}' if self.vnc_kind.currentIndex()==0 else f'127.0.0.1:{port}'
        success,_=QProcess.startDetached(path,[address])
        if not success:
            self.add_log('VNC 软件启动失败，请检查所选 exe 路径')
        else:
            self.add_log('已打开 VNC；树莓派须已登录图形桌面且 WayVNC 正常运行')

    def set_direction(self,direction):
        """@brief 更新按住方向，松开时立即发送零速度。"""
        self.direction=direction if self.server_armed else (0.,0.)
        if direction==(0,0):
            self.send('motion',vx=0.,wz=0.)

    def disarm(self):
        """@brief 撤销使能，不依靠后续界面计时器才发送停止。"""
        self.direction=(0.,0.)
        self.server_armed=False
        self.send('disarm')

    def application_state(self,state):
        """@brief 窗口失去前台状态时撤销遥控，防止按钮释放事件丢失。"""
        if state!=Qt.ApplicationActive and not self.navigation_mode:
            self.disarm()

    def clear_imu(self):
        """@brief 切换来源时清空旧角度，避免来源标签与历史值不一致。"""
        self.attitude.angles=None
        self.attitude.update()
        self.imu_values.setText('Roll —     Pitch —     Yaw —')
        self.stamps.pop('imu',None)

    def request_yaw(self, save=None):
        """@brief save为None时读取，False仅应用，True应用并保存树莓派配置。"""
        if not self.worker or not self.worker.ready:
            return
        if save is not None and (self.server_armed or self.navigation_mode or not self.yaw_loaded):
            self.yaw_status.setText('请先禁能小车并读取当前参数')
            return
        self.yaw_pending=True
        self.yaw_status.setText('正在读取…' if save is None else '正在提交参数…')
        if save is None:
            self.send('yaw_get')
        else:
            self.send('yaw_set',values={key:editor.value() for key,editor in self.yaw_fields.items()},save=save)

    def show_imu(self,data):
        """@brief 仅用当前选定来源更新姿态，原始量保持自身单位。"""
        self.stamps['imu']=time.monotonic()
        valid=data.get('valid',False)
        self.attitude.angles=data.get('rpy_deg') if valid else None
        self.attitude.update()
        if valid:
            r,p,y=data['rpy_deg']
            self.imu_values.setText(f'Roll {r:.2f}°     Pitch {p:.2f}°     Yaw {y:.2f}°')
        else:
            self.imu_values.setText('姿态无效 / 未提供解算结果')
        self.imu_detail.setText(json.dumps(data,ensure_ascii=False,indent=2))

    def on_message(self,data):
        """@brief 按类型更新真实数据；接收时记录本机时间用于超时提示。"""
        kind=data.get('type')
        now=time.monotonic()
        if kind in ('system','wheels','scan','map','navigation_scene'):
            self.stamps[kind]=now
        if kind=='hello':
            self.yaw_loaded=False
            self.request_yaw()
        elif kind=='yaw_parameters':
            self.yaw_pending=False
            values=data.get('values')
            if (data.get('success') or data.get('applied')) and isinstance(values,dict) and all(k in values for k in self.yaw_fields):
                for key,editor in self.yaw_fields.items():
                    editor.blockSignals(True)
                    editor.setValue(values[key])
                    editor.blockSignals(False)
                self.yaw_loaded=True
                self.yaw_current.setText('当前生效：'+ ' · '.join(
                    f'{key}={values[key]:.4f}' for key in self.yaw_fields))
            self.yaw_status.setText(data.get('text','参数响应异常'))
            self.add_log(data.get('text','参数响应异常'))
        elif kind=='log':
            self.add_log(data.get('text',''))
        elif kind=='navigation':
            self.navigation_mode=bool(data.get('active'))
            self.navigation_ready=bool(data.get('ready'))
            distance=data.get('distance_remaining')
            suffix=f' · 剩余{distance:.2f}m' if isinstance(distance,(int,float)) else ''
            self.navigation_status.setText(data.get('text','')+' · '+(data.get('reason') or '定位与底盘就绪')+suffix)
        elif kind=='navigation_scene':
            stamp=data.get('server_time',time.time()+getattr(self.worker,'clock_offset',0.0))
            age=time.time()+getattr(self.worker,'clock_offset',0.0)-stamp
            self.map_view.set_scene(data,age)
        elif kind=='map_command_result':
            self.map_notice(('树莓派已处理：' if data.get('success') else '未执行：')+data.get('text',''))
        elif kind=='system':
            temp=data.get('temperature')
            self.system_info.setText(f"主机：{data.get('hostname')}\nCPU 温度：{temp if temp is not None else '未知'} °C\n"
                f"1 分钟负载：{data.get('load',0):.2f}\nROS 订阅：{'可用' if data.get('ros') else '不可用'}")
        elif kind=='ports':
            previous=self.serial_ports.currentText()
            self.serial_ports.clear()
            self.serial_ports.addItems(data['ports'])
            # 刷新时保留已有选择，首次连接默认选择优先列出的底盘 UART3。
            if data['ports']:
                index=self.serial_ports.findText(previous)
                self.serial_ports.setCurrentIndex(index if index>=0 else 0)
        elif kind=='wheels':
            self.wheel_control=bool(data.get('wheel_control'))
            extra=data.get('extra',{})
            self.servo_capable=bool(extra.get('capabilities',0)&2)
            self.servo_allowed=bool(data.get('servo_allowed'))
            self.servo_pulse.setRange(data.get('servo_min_us',1000),data.get('servo_max_us',2000))
            pulse=extra.get('servo_pulse_us',0)
            self.servo_status.setText(
                f'当前输出：{pulse} μs · 确认序号 {extra.get("servo_ack",0)}' if pulse else
                '输出关闭' if self.servo_capable and self.servo_allowed else
                '未解锁：请先确认型号，并在树莓派配置中开启' if self.servo_capable else '固件未提供舵机接口')
            self.speed_plot.append(now,data['speeds'])
            for row,(target,actual) in enumerate(zip(data['targets'],data['speeds'])):
                for col,value in enumerate([target,actual,target-actual],1):
                    self.wheel_table.item(row,col).setText(f'{value:.3f}')
                counts=extra.get('counts',[])
                count=counts[row] if len(counts)==4 else None
                if count is not None and count>=0x80000000:
                    count-=0x100000000
                self.wheel_table.item(row,4).setText(str(count) if count is not None else '—')
            if self.imu_source.currentIndex()==0:
                self.show_imu({'source':'STM32','valid':data['imu_valid'],'rpy_deg':data['rpy_deg'],
                    'gyro_raw':data['gyro_raw'],'accel_raw':data['accel_raw']})
            fault_names=[name for bit,name in enumerate(('欠压','曾发生串口接收错误','命令超时','IMU未就绪'))
                         if extra.get('faults',0)&(1<<bit)]
            self.wheel_age.setText(f"固件 {data['firmware']} · MCU {data['tick']} ms · 电压 {data['voltage']:.2f} V · "
                f"底盘{'已使能' if data['armed'] else '禁能'} · {'超时停车' if data['timeout'] else '正常反馈'} · "
                f"航向来源 {data.get('yaw_source','stm32')} · 扩展状态：{('、'.join(fault_names) or '正常') if extra else '未提供'}")
        elif kind=='imu' and data['source']==self.imu_source.currentText():
            self.show_imu(data)
        elif kind=='state':
            self.server_armed=bool(data['armed'])
            for name,label in self.program_labels.items():
                programs=data['programs']
                label.setText('进程运行' if name in programs and programs[name] is None else
                              f'已退出 ({programs[name]})' if name in programs else '本会话未启动')
        elif kind=='control':
            self.server_armed=bool(data['armed'])
        elif kind=='scan':
            self.scan_view.points=data['points']
            self.scan_view.update()
        elif kind=='map':
            self.map_view.set_map(data)
        elif kind=='result':
            source=data['source']
            self.stamps[source]=now
            result=data['data']
            if source=='pi':
                self.pi_results.setPlainText(json.dumps(result,ensure_ascii=False,indent=2))
            elif source=='k230':
                detections=result.get('detections',[])
                if not isinstance(detections,list):
                    self.add_log('K230 detections 字段必须为数组')
                    return
                self.k230_info.setText(f"K230 · 帧 {result.get('frame_id','—')} · FPS {result.get('fps','—')} · 检出 {len(detections)} 个目标")
                self.detections.setRowCount(min(len(detections),100))
                for row,item in enumerate(detections[:100]):
                    if not isinstance(item,dict):
                        continue
                    for col,key in enumerate(['class_id','label','cx','cy','confidence']):
                        self.detections.setItem(row,col,QTableWidgetItem(str(item.get(key,'—'))))
        if self.recorder and kind in ('wheels','imu','result','system'):
            try:
                self.recorder.writerow([datetime.now().isoformat(timespec='milliseconds'),kind,
                    data.get('source',''),*data.get('speeds',['']*4),*data.get('targets',['']*4),
                    *data.get('rpy_deg',['']*3),json.dumps(data,ensure_ascii=False)])
            except OSError as error:
                self.add_log(f'记录失败：{error}')
                self.stop_record()

    def tick(self):
        """@brief 更新新鲜度并发送有限期遥控，界面卡顿时不续发旧命令。"""
        now=time.monotonic()
        connected=bool(self.worker and self.worker.ready)
        self.map_view.set_connected(connected)
        if not connected:
            self.yaw_loaded=False
            self.yaw_pending=False
            self.navigation_mode=False
            self.navigation_ready=False
        map_available=connected and self.map_view.map_geometry is not None and self.map_view.map_geometry.frame=='map'
        scene_fresh=now-self.map_view.scene_received<.8 and self.map_view.scene.get('pose') is not None
        self.navigation_allow.setEnabled(map_available and scene_fresh and self.navigation_ready and not self.navigation_mode)
        self.map_mode_buttons['initial'].setEnabled(map_available and not self.navigation_mode and not self.server_armed)
        goal_allowed=map_available and scene_fresh and self.navigation_mode and self.navigation_ready
        self.map_mode_buttons['goal'].setEnabled(goal_allowed)
        if (self.map_view.mode=='goal' and not goal_allowed) or (self.map_view.mode=='initial' and not self.map_mode_buttons['initial'].isEnabled()):
            self.map_view.set_mode('pan')
        self.yaw_read.setEnabled(connected and not self.yaw_pending)
        tuning_allowed=connected and self.yaw_loaded and not self.server_armed and not self.navigation_mode and not self.yaw_pending
        self.yaw_apply.setEnabled(tuning_allowed)
        self.yaw_save.setEnabled(tuning_allowed)
        for editor in self.yaw_fields.values():
            editor.setEnabled(tuning_allowed)
        for key,label in self.health.items():
            age=now-self.stamps.get(key,-1e9)
            fresh=connected and age<1.5
            label.setText('在线' if fresh else '数据超时' if key in self.stamps else '未接入')
            label.setStyleSheet('font-size:22px;color:'+('#52d6c7' if fresh else '#8fa5bf'))
        wheel_fresh=connected and now-self.stamps.get('wheels',-1e9)<.25
        if not wheel_fresh:
            self.server_armed=False
            self.direction=(0.,0.)
        self.arm_btn.setEnabled(wheel_fresh and self.wheel_control and not self.server_armed and not self.navigation_mode)
        self.servo_apply.setEnabled(wheel_fresh and self.servo_capable and self.servo_allowed)
        self.servo_release.setEnabled(wheel_fresh and self.servo_capable)
        self.motion_status.setText('自主控制中 · 遥控已禁用' if self.navigation_mode else
                                  '已使能 · 按住方向按钮运行' if self.server_armed else
                                  '旧固件仅可监测，请先更新底盘固件' if wheel_fresh and not self.wheel_control else
                                  '底盘已连接 · 禁能' if wheel_fresh else '等待 wheels-v2 固件反馈；默认禁能')
        for b in self.arrow_buttons:
            b.setEnabled(wheel_fresh and self.server_armed and not self.navigation_mode)
        if self.server_armed and not self.navigation_mode:
            if not self.isActiveWindow() or self.pages.currentIndex()!=0:
                self.disarm()
            else:
                self.send('motion',vx=self.direction[0]*self.vx.value(),wz=self.direction[1]*self.wz.value())
        def age_text(key):
            return f'{now-self.stamps[key]:.1f} 秒前' if key in self.stamps else '未接入'
        self.spatial_status.setText(f"Windows本地渲染 · 激光：{age_text('scan')} | 地图：{age_text('map')} | 定位叠加：{age_text('navigation_scene')} · 无有效TF时隐藏叠加")
        if self.record_file:
            try:
                self.record_file.flush()
            except OSError as error:
                self.add_log(f'CSV 写入失败：{error}')
                self.stop_record()

    def add_log(self,text):
        """@brief 显示带本机时间的事件，日志容量由控件统一限制。"""
        line=datetime.now().strftime('%H:%M:%S')+'  '+str(text)[:12000]
        self.log.appendPlainText(line)
        self.overview_log.appendPlainText(line)

    def toggle_record(self):
        """@brief 记录真实接收的数据为 UTF-8 BOM CSV，便于 Excel 打开。"""
        if self.record_file:
            self.stop_record()
            return
        path,_=QFileDialog.getSaveFileName(self,'保存实时数据','robot_data_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv','CSV (*.csv)')
        if path:
            try:
                self.record_file=open(path,'w',encoding='utf-8-sig',newline='')
                self.recorder=csv.writer(self.record_file)
                self.recorder.writerow(['本机接收时间','类型','来源','实测A_mps','实测B_mps','实测C_mps','实测D_mps',
                    '目标A_mps','目标B_mps','目标C_mps','目标D_mps','roll_deg','pitch_deg','yaw_deg','完整JSON'])
                self.record_btn.setText('停止 CSV 记录')
                self.record_status.setText(path)
            except OSError as error:
                self.stop_record()
                QMessageBox.warning(self,'记录失败',str(error))

    def stop_record(self):
        """@brief 关闭记录文件并恢复按钮状态。"""
        if self.record_file:
            try:
                self.record_file.close()
            except OSError:
                pass
        self.record_file=None
        self.recorder=None
        self.record_btn.setText('开始 CSV 记录')
        self.record_status.setText('记录已停止')

    def export_log(self):
        """@brief 将当前界面日志保存为 UTF-8 文本。"""
        path,_=QFileDialog.getSaveFileName(self,'导出日志','robot_console.log','日志 (*.log)')
        if path:
            try:
                Path(path).write_text(self.log.toPlainText(),encoding='utf-8')
            except OSError as error:
                QMessageBox.warning(self,'保存失败',str(error))

    def closeEvent(self,event):
        """@brief 关闭窗口先撤销遥控并异步断开，避免销毁仍在运行的线程。"""
        self.disarm()
        self.stop_record()
        if self.worker:
            self.closing=True
            self.worker.stop()
            self.setEnabled(False)
            self.connection_status.setText('正在安全断开，请稍候…')
            event.ignore()
        else:
            event.accept()


def run():
    """@brief 初始化应用样式并进入 Qt 事件循环。"""
    app=QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    app.setFont(QFont('Microsoft YaHei UI',10))
    window=Window()
    window.show()
    sys.exit(app.exec())
