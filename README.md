# 2027 光电小车

STM32 四轮底盘、树莓派 ROS2 控制程序和 Windows 调试上位机的队内共享仓库。

## 目录与分工

| 目录 | 用途 | 运行位置 |
| --- | --- | --- |
| `10.8/` | 编码器与板载 IMU 采集、四轮速度 PI、PWM、串口协议和失联保护 | STM32 |
| `robot_console/desktop/` | 遥控、调参、地图和导航界面 | Windows |
| `robot_console/pi/` | SSH 网关、底盘节点、航向控制、里程计和导航接口 | 树莓派 ROS2 容器 |
| `robot_console/shared/` | 通信协议和公共计算模块 | Windows / 树莓派 |
| `ros2_lidar/` | DELTA2C 雷达驱动、Cartographer 建图配置和验证工具 | 树莓派 ROS2 容器 |
| `ros2_servo/` | 舵机角度到 STM32 PA8 指令的 ROS 桥接 | 树莓派 ROS2 容器 |

控制链路：Windows 上位机 → SSH 网关 → 树莓派 ROS2 控制节点 → UART3 → STM32。
树莓派以 50 Hz 计算轮速目标，STM32 以 100 Hz 执行四轮速度 PI。

## 开发入口

- STM32：使用 Keil 打开 `10.8/MDK-ARM/WHEELTEC.uvprojx`，自行编译和烧录当前源码。
- Windows：进入 `robot_console/`，执行 `build.ps1` 生成 EXE；依赖由脚本安装。详见该目录 README。
- 树莓派：见 [部署说明](docs/树莓派部署.md)。
- 当前底盘接口：[COMMUNICATION.md](10.8/COMMUNICATION.md)、[PROTOCOL.md](robot_console/PROTOCOL.md)。
- 控制参数：[PI_CONTROL.md](robot_console/PI_CONTROL.md)。
- 导航操作与验收范围：[NAVIGATION.md](robot_console/NAVIGATION.md)。
- 舵机接口：[ros2_servo/README.md](ros2_servo/README.md)。

## 当前状态与协作约定

仓库保存本地源码快照，不包含 Windows 虚拟环境、EXE、旧 HEX、临时备份和编译输出。各模块文档包含不同日期的历史记录，源码实现、远端部署状态和实车验收需要分别确认。已有导航定位与规划静态验证记录，自主行驶尚未完成实车验收。

新旧底盘协议不能混用：当前树莓派控制对应 wheels-v2 固件。编译或打包成功不等于已经烧录、部署或通过实车验证。

修改协议时同步维护 STM32、树莓派、上位机和协议文档。树莓派实际参数可能与仓库默认参数不同，更新前保存设备上的调参结果。

第三方驱动保留原有许可文件；整个工程尚未设置统一开源许可证。私钥、密码和设备认证文件不得提交。
