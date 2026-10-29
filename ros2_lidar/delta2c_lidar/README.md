# DELTA2C Pro-D-V001 ROS 2 驱动

适用环境：树莓派 5、Debian 13 宿主机、Ubuntu 22.04 / ROS 2 Humble 容器。

## 当前接线

GPIO 使用 BCM 编号，括号中为 40 针排针的物理脚号。

| 雷达端标识 | 连接位置 |
| --- | --- |
| TX | 树莓派 GPIO15 / RXD0（物理 10 脚） |
| GND | 树莓派 GND（例如物理 6 脚），并与供电电源地共地 |
| VCC | 独立稳定的 5 V 逻辑供电 |
| M+ / M− | 独立可调电机供电，按实物标识接正负极 |

不要根据线色或连接器方向猜引脚顺序。雷达供电为 5 V 不代表 TX 电平为 5 V；树莓派 GPIO 仅接受 3.3 V。接线前需确认 TX 高电平不超过 3.3 V，否则加电平转换。不得从 GPIO 或树莓派的 3.3 V 引脚直接给电机供电。

原仓库实测电机约 2.6 V 时接近 7 Hz，约 2.9 V 时接近 7.9 Hz；不同个体有差异。这是调试参考，并非制造商保证的电压范围。只输出 0xAE 转速帧时，检查电机转速是否超出测距工作窗口。调整供电需由用户操作，驱动不控制电机或 GPIO12 舵机。

GPIO14（物理 8 脚）为 TXD0，本雷达只有 TX 数据输出，不需要连接 GPIO14。

## 树莓派 UART

已在 `/boot/firmware/config.txt` 的 `[pi5]` 段写入：

```ini
dtoverlay=uart0-pi5
```

当前会话已通过动态 overlay 启用，开机配置将在下次启动加载。实际设备是 `/dev/ttyAMA0`；Pi 5 的 `/dev/serial0` 在此系统中指向调试口 `/dev/ttyAMA10`，不要混用。

## 构建和运行

宿主机工作区为 `/home/xu/ros2_ws`，包位于 `src/delta2c_lidar`。先在宿主机运行 `ros2-shell`，再在容器内运行：

```bash
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select delta2c_lidar
source install/setup.bash
ros2 run delta2c_lidar lidar_node
```

若现有开发容器缺少串口组权限，运行前用 `id` 和 `ls -l /dev/ttyAMA0` 检查；容器进程需要加入设备的 `dialout` 组。此项目的 `ros2-shell` 已补充该附加组。

按 Ctrl+C 停止，释放串口；默认不设置 ROS 节点开机自启。

另开一个终端进入 `ros2-shell`，可查看状态和扫描：

```bash
ros2 topic echo /diagnostics
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic hz /scan
```

`/scan` 类型为 `sensor_msgs/msg/LaserScan`，使用 SensorData QoS，`frame_id=laser`。RViz2 将 Fixed Frame 设置为 `laser`，LaserScan 显示项选择 `/scan` 并使用 Best Effort。驱动未提供尚未测量的 `base_link → laser` 安装变换。

## 参数和数据规则

```bash
ros2 run delta2c_lidar lidar_node --ros-args \
  -p port:=/dev/ttyAMA0 -p baudrate:=115200 \
  -p clockwise:=true -p angle_offset_deg:=0.0
```

- 支持协议版本 0x11、0x13，115200 8N1，长度关系和 16 位累加校验全部通过才接收。
- 0xAE 仅提供转速；0xAD 才提供角度与距离。不能把 0xAE 当作扫描。
- 距离按原始值 × 0.00025 米换算；强度为零、距离为零、原始距离超过 16000 视作无回波。
- 默认 360 个角度槽，`range_min=0.05`、`range_max=4.0` 米为软件过滤配置，不是标定结果。缺测槽为正无穷。
- 首次不完整的一圈会丢弃；以 10 度扇区统计角度覆盖，默认 `min_coverage=0.8`，断流或覆盖不足的多圈残片不会拼成完整扫描。
- `clockwise` 和 `angle_offset_deg` 用于安装方向；默认顺时针映射，必须用前方、左方的实物验证角度和零点后再用于导航。
- 扫描时间来自接收时钟，时间戳是接收时间估计。无逐点硬件时间戳，`time_increment=0`，不适合依赖精确逐点时间的高速运动去畸变。
- `/diagnostics` 每秒发布无串口、无数据、协议不匹配、仅转速、测距不连续和正常扫描等状态，串口异常每 2 秒重试。

## 参考资料

- [hapynfdj/delta2c-lidar](https://github.com/hapynfdj/delta2c-lidar)：协议、真实采样、接线与电机工作窗口；MIT。
- [kaiaai/lds2d](https://github.com/kaiaai/lds2d)：Python 型号解码器和 0x11/0x13 兼容性；Apache-2.0。
- [kaiaai/LDS](https://github.com/kaiaai/LDS)：Delta 型号差异及距离比例；Apache-2.0。
- [树莓派官方 UART 文档](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-device-tree-overlays-for-uarts)。

本包按协议重新实现流解析与 ROS 节点，不启动上游库的电机控制。型号比较工具使用未修改的上游解码器离线回放同一采样文件，比较结果不代表把物理雷达改成其他型号。
