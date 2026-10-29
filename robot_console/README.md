# 2026-10-29 Windows本地地图与导航显示

新版“地图与导航”在Windows绘制地图、真实激光、车体位置及规划路径，支持滚轮/按钮缩放、平移、拖动设置初始位置和导航目标。树莓派继续运行ROS2定位、规划与底盘执行，日常无需启动RViz或VNC。目标直接调用NavigateToPose，保留自主授权、过期拒绝、取消停车和停稳确认。缺少TF或实时数据时隐藏对应叠加，不补造位置。详见[NAVIGATION.md](NAVIGATION.md)。本轮只完成本地开发、离线测试与打包；电脑已离开原树莓派网段，未做在线联调或实车运动。新版首次连接将同步新增接口。

# 2026-10-29 地图定位导航更新

树莓派现已安装Nav2、AMCL和地图服务，上位机“雷达 / RViz”新增停止建图、启动地图定位导航、打开导航RViz、允许自主控制及取消停车入口。按25×20厘米车体和暂定零雷达偏移配置，限速0.08米/秒。地图加载、定位、路径规划、零速授权与取消已进行静态验证，尚未执行自主行驶验收。详见[NAVIGATION.md](NAVIGATION.md)。电脑已有RealVNC Viewer，使用导航版exe，无需安装Windows版ROS2。下列记录中的“未部署”和“Nav2未接入”属于此前版本历史。

# 2026-10-29 上位机调参更新

“IMU 姿态”页可读取当前yaw外环参数、应用本次修改或应用并保存。需先禁能小车，修改对象为树莓派ROS节点；调参本身不需要烧录STM32。详见 [PI_CONTROL.md](PI_CONTROL.md)。本次没有部署树莓派。

# 2026-10-29 本地更新

已加入累计编码器反馈、树莓派 `/odom`、板载/外接IMU航向来源选择、PA8舵机接口及故障状态。舵机失联保持最后输出，上电关闭；型号未确认，树莓派 `servo_allowed=false` 默认锁定。使用说明见 [PI_CONTROL.md](PI_CONTROL.md)。本次未部署、未编译或烧录STM32。

# 当前版本：车载树莓派控制（2026-10-28）

本版已将 yaw 闭环与四轮目标分配迁到树莓派 ROS2，STM32 只执行四轮速度 PI、采集与停车保护。**需要配套 wheels-v2 新固件**；旧 console-v1 仅允许监测和禁能。本次未部署、未编译或烧录 STM32，尚未实车验收新架构。

最新使用方式、参数和 ROS 接口见 [PI_CONTROL.md](PI_CONTROL.md)。用户后续点击新版上位机的连接按钮时才同步树莓派程序，参数文件已存在时不覆盖。旧 dist/STM32_PC10_PC11_UART3.hex 不包含本次修改，不能作为新固件使用。

下文为旧版功能与开发历史记录；旧协议和旧控制路径描述以 PI_CONTROL.md 和 PROTOCOL.md 的当前接口为准。

---

# 小车调试台

## UART3 更新（2026-10-27）

树莓派底盘接口已改为 GPIO8/9 的 UART3，设备 `/dev/ttyAMA3`；STM32 使用 PC10/PC11 的 USART3，115200/8N1。GPIO4/5 已按用户确认留给雷达。接线和 FlyMcu 烧录步骤见 `UART3_接线与烧录.md`。

本次已完成固件编译、串口协议和界面选择检查，并在树莓派在线启用 UART3、保存开机配置。上位机将 UART3 优先列出，拒绝打开雷达 UART0/UART2 与调试口。当前遥控沿用网关直接串口路径，尚未改为 ROS `/cmd_vel` 控制。

固件编译不等于已烧录或实车验收；下文“本次未编译”等描述指 10 月 25 日的原始交付记录，以此更新和接线说明为准。原文件已备份到 `firmware/before_uart3_20261027.zip`。

Windows 桌面上位机，本地开发版。双击 `dist/RobotConsole.exe` 启动，Windows 无需另装 Python 或 ROS 2。

**按用户要求：只完成本地开发和 exe 打包，未进行运行测试、远程部署、固件编译/烧录或硬件验证。** 所有连接、遥控、数据接入和 RViz 功能均需后续联调，不能将源码实现视为实车验收通过。

## 页面与功能

| 页面 | 功能 |
| --- | --- |
| 总览与遥控 | SSH 连接、系统温度/负载、数据新鲜度、底盘 UART3 / USB TTL 选择、禁能/使能、按住式方向控制 |
| 四轮速度 | A/B/C/D 目标、实测、误差及最近十秒实测曲线 |
| IMU 姿态 | Roll/Pitch/Yaw、车体姿态线框、原始数据，支持 STM32 或 ROS IMU 来源 |
| 计算结果 | K230 类别、中心像素坐标、置信度、FPS；树莓派任意 JSON 结果 |
| 雷达 / RViz | 分开显示真实扫描和地图；启动/停止 RViz；通过 SSH VNC 查看完整 RViz |
| 程序与日志 | 启停预设程序，显示退出码、事件及程序日志 |

底部支持 CSV 记录和禁能。未收到数据时显示“未接入”，接收中断后显示“数据超时”；旧数值可能保留以便查看历史，不代表当前有效数据。没有自动生成的模拟数据。

## 现在如何打开

双击 `dist/RobotConsole.exe`。可在未连接状态下查看页面，不操作硬件。

首次打包程序提取 Qt 运行库需要一点时间。整个 exe 包含界面运行时和网关源码，不要求用户配置 Python。运行日志和 CSV 通过界面选择保存位置；密码不保存到设置或文件。地址、用户名、端口和 VNC Viewer 路径保存在当前用户的 Windows 设置中。

## 后续连接树莓派

以下为待联调步骤，本次没有在树莓派执行：

1. 树莓派可从 Windows 访问，SSH 正常运行。先在终端 `ssh xu@192.168.10.193` 核对真实主机指纹并保存。上位机使用当前用户 `.ssh/known_hosts`，拒绝未知或变化的主机密钥。
2. 保留现有 `ros2-humble` 容器，容器映射宿主机 `~/ros2_ws` 到 `/workspace/ros2_ws`，并具备 ROS Humble、rclpy、python3-serial，以及访问 USB 串口的权限。现有环境采用 privileged 容器，本版没有重新创建它。
3. 登录用户须有 sudo 权限，当前连接表单使用同一密码做 SSH 和 sudo 认证。SSH 密钥登录或独立 sudo 密码尚未实现。
4. 填入地址、用户名和密码，点“连接树莓派”。上位机通过 SFTP 上传网关到 `~/ros2_ws/robot_console`，然后启动容器并在容器内运行网关。
5. 界面收到网关握手后显示已连接。连接操作不会自动打开底盘串口、不会自动使能电机，也不会自动启动雷达或 SLAM。

通信直接使用 SSH exec 的加密双向通道；VNC 使用同一 SSH 连接的本地回环临时端口转发，不额外开放公网 ROS/WebSocket 端口。SSH 本身不解决不同局域网的网络可达性，跨网仍需 VPN 或跳板网络。

本版一次只允许一个网关会话，断开后会禁能底盘，并停止**本会话启动**的雷达、建图及 RViz。其他终端启动的程序不由本上位机接管。强制结束电脑进程或断网时，底盘应由固件超时停车；远程进程资源回收依赖 SSH 断开被系统检测，仍需后续验证。

## 如何启动已写好的 ROS 程序

在“程序与日志”或“雷达 / RViz”页点对应启动按钮：

| 预设 | 容器内命令 |
| --- | --- |
| 雷达 | `ros2 run delta2c_lidar lidar_node` |
| 纯雷达建图 | `ros2 launch /workspace/ros2_ws/lidar_support/mapping/mapping.launch.py rviz:=false` |
| RViz | `rviz2 -d /workspace/ros2_ws/lidar_support/mapping/mapping.rviz` |
| 保存地图 | `python3 /workspace/ros2_ws/lidar_support/mapping/save_map.py /workspace/ros2_ws/maps/console_map` |

这些路径沿用此前的树莓派工作区。本版不重新安装或构建雷达、Cartographer、RViz。重复启动检查仅覆盖已知雷达和 Cartographer 节点；其他程序是否占用设备仍需操作者确认。地图保存会覆盖同名 `console_map` 文件，本版不提供地图下载按钮。

新增预设程序时，开发者在 `pi/gateway.py` 的 `programs` 白名单登记命令列表，并在桌面页面添加对应按钮，然后重新打包。不从界面接收任意 shell 命令。节点需预先构建并在工作区 `install/setup.bash` 中可发现。

## RViz 如何使用

1. 树莓派须已登录图形桌面，现有 WayVNC 服务和 XWayland `:0` 正常，`~/.Xauthority` 可用。
2. 连接时网关复制桌面授权文件到工作区专用文件，以 600 权限保存；RViz 使用该文件访问现有显示器。
3. 点“启动 RViz”，RViz 窗口在**树莓派桌面**运行；不会把 Linux RViz 原生嵌入 Windows Qt 窗口。
4. Windows 安装 TigerVNC Viewer 或 RealVNC Viewer 后，点“选择 VNC 软件”，选择其 Viewer exe，并选择对应类型。
5. 点“通过 VNC 查看 RViz”，上位机在本机回环临时端口转发树莓派 `127.0.0.1:5900`，调用 Viewer 打开。VNC 登录按树莓派桌面服务配置操作，上位机不会保存其密码。

普通监控可只看上位机的扫描与地图。完整 RViz 用于 TF、点云、地图及后续机器人模型调试。启动 RViz 时，若检测到 Cartographer 建图节点则使用 `map` 配置，否则使用随上位机提供的 `pi/lidar.rviz`，以 `laser` 为固定和视图坐标系显示 `/scan`；切换用途后需停止并重新启动 RViz。当前软件渲染会占用树莓派 CPU；本次未测量性能。Nav2 未接入，导航目标发送按钮尚未实现。

查看 VNC 会令本机上位机失焦，因此先撤销其遥控；不支持在 VNC 窗口获得焦点时继续按本机按钮运动。

## 底盘源码修改

在现有 `../10.8` 工程中新增 `Core/Inc/chassis_link.h`、`Core/Src/chassis_link.c`，并集成到 main、USART3、100 Hz 控制中断和 Keil 工程。原四个集成文件备份于 `firmware/original_before_console.zip`。

- 上电零目标、STBY 禁能；USART3 改为独占新版二进制协议，不再输出 VOFA 文本，也不再接受旧 11 字节命令。
- 默认禁能；会话同步后，收到有效零速度使能命令才能接管。
- 100 Hz 控制中断检查 300 ms 有效命令超时，超时撤销 STBY，清除轮速目标及 PI 历史。
- 四轮编码器和原有 PI 闭环保留，遥控模式绕过旧直行/相对转向状态机；本版不提供蓝牙运动控制入口。
- 轮距初值沿用 `Wheel_spacing=0.130 m`，差速分配 `vR=vx+wz*b/2`、`vL=vx-wz*b/2`。四轮滑移转向的有效轮距需要实测标定；本次未验证轮序、方向和距离精度。
- 串口 RX 中断只写缓冲，主循环解析；反馈使用非阻塞 UART 中断发送，未改为 DMA。没有实测中断耗时或控制性能。
- 仍依赖原工程 IMU 初始化与静止校准；IMU 初始化失败时可能停在原初始化循环，保持上电零 PWM，无法进入正常反馈。没有增加硬件独立看门狗；100 Hz 软件超时不能覆盖 CPU 卡死、失电或驱动硬件故障。

**未编译、未烧录、未验证停车距离。不要将当前 exe 与旧固件搭配当作已具备失联停车；旧固件不会通过本版反馈握手，遥控按钮不会开放。** 后续应先离地架车验证上电不动、方向与松手/失焦/拔线/断网停车，再进行低速地面测试。

底盘 GPIO UART3：Pi GPIO8/TX（物理24脚）→PC11/RX，Pi GPIO9/RX（物理21脚）←PC10/TX，GND共地。设备为 /dev/ttyAMA3；GPIO4/5 按现有实物接线留给雷达。仅接3.3V逻辑信号与地，独立供电时不互接电源。

## IMU、树莓派和 K230 接口

- IMU 页面可选 STM32 板载姿态、`/imu/data`、`/imu/data_raw`。ROS 消息为 `sensor_msgs/Imu`；原始消息不带有效姿态时只显示原始量，姿态图保持无效状态。新 I2C IMU 型号仍未确定，未编写其硬件驱动。
- 树莓派计算结果发布到 `/pi/results_json`，类型 `std_msgs/String`，内容为 JSON 对象，例如定位质量和算法耗时。
- K230 预留独立 USB TTL 串口，不采集摄像头画面；字段和坐标约定见 `PROTOCOL.md`。`pi/k230_bridge.py` 提供接收和校验代码，但尚未接真实 K230 或模型，也未注册自动启动项。

串口桥后续手动启动示例（容器内已 source ROS 环境，端口换为实物对应路径）：

```bash
python3 /workspace/ros2_ws/robot_console/pi/k230_bridge.py --port /dev/serial/by-id/实际K230串口 --baud 115200
```

## 重新打包

QtCore 加载修复：固定 PySide6-Essentials/Shiboken 为配套的 6.8.3，构建时隔离其他软件的 PATH，避免误收 Poppler 的同名 `icuuc.dll`。构建结束运行 `tools/check_native_dependencies.py`，静态核对 Qt 原生 DLL 来源及导入符号；该检查不启动界面，不进行硬件测试。

Windows PowerShell 在此目录执行：

```powershell
.\build.ps1
```

构建会创建本目录 `.venv` 并安装固定版本依赖，然后生成单文件 exe。Qt/PySide6、Paramiko、PyInstaller 等第三方依赖有各自许可，文本保留在 `dist/第三方许可`；重新分发时应一并保留相应许可。源码、协议与网关均随项目保留。

开发参考：[Qt for Python 打包说明](https://doc.qt.io/qtforpython-6/deployment/deployment-pyinstaller.html)、[Paramiko 主机密钥与连接接口](https://docs.paramiko.org/en/stable/api/client.html)、[TigerVNC Viewer 地址格式](https://tigervnc.org/doc/vncviewer.html)。
