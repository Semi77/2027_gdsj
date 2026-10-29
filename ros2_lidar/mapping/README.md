# 纯雷达二维建图

本配置针对小车上固定安装的 DELTA2C Pro-D-V001，使用真实 `/scan`。RViz2 负责显示，Cartographer 负责生成地图。

## 启动

先确认雷达驱动正在运行；诊断应为“雷达测距与整圈扫描正常”。雷达进程没有运行时，先在 `ros2-shell` 中启动：

```bash
ros2 run delta2c_lidar lidar_node
```

在另一个 SSH 终端运行宿主机命令 `ros2-map`，在树莓派桌面/VNC 中查看 RViz2。

本次测试只使用雷达，`tracking_frame` 和 `published_frame` 都是 `laser`，由 Cartographer 发布 `map → odom → laser`。没有伪造轮速里程计，也不假定尚未测量的雷达相对车身安装偏移。

保持雷达扫描平面水平，手动缓慢移动小车，观察地图扩展。当前驱动没有逐点硬件时间戳，无里程计与 IMU 的纯雷达建图应避免快速旋转、倾斜和突然加速；小范围生成地图不等同于完成整屋或导航精度验证。

## 显示

RViz2 已配置 Fixed Frame 为 `map`，显示 `/map`、`/scan` 和 TF。`/scan` 使用 Best Effort，`/map` 使用 Reliable / Transient Local。

使用现有树莓派 XWayland 的本地抽象套接字，不开放额外 X11 TCP 端口。显示授权文件为用户私有文件，未关闭 X11 访问控制。

## 保存当前地图

在容器终端中运行：

```bash
python3 /workspace/ros2_ws/lidar_support/mapping/save_map.py \
  /workspace/ros2_ws/maps/lidar_map
```

将保存 PGM、YAML 和栅格统计 JSON。保存前检查 `/map` 确实包含空闲和占用栅格；没有地图时明确超时报错。

建图配置默认不在树莓派开机后自动运行。停止建图可在 `ros2-map` 所在终端按 Ctrl+C，雷达进程独立运行。

参考：[Cartographer 配置说明](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)、[ROS 2 Cartographer](https://github.com/ros2/cartographer_ros)。
