# 建图验证结果

2026-10-25 已在树莓派 5 上启动 ROS 2 Humble Cartographer 2D 与 RViz2，使用 DELTA2C Pro-D-V001 的真实 `/scan`。

- RViz2 实际桌面截图为 `rviz_mapping.png`，Global Status 显示 Ok，能够显示灰度栅格地图与实时红色激光点。
- 首次保存的地图为 94×96 个栅格，分辨率约 0.05 米，包含 170 个占用栅格、1545 个空闲栅格和 6649 个未知栅格。
- 地图文件在树莓派 `/home/xu/ros2_ws/maps/lidar_test.pgm` 与 `.yaml`；本地副本在 `ros2_lidar/maps/`。
- 使用雷达坐标系 `laser` 跟踪，由实际扫描匹配生成 `map → odom → laser`，未伪造轮速里程计或安装变换。
- 安装并运行 RViz2 和 Cartographer；宿主机启动入口为 `~/bin/ros2-map`，配置在 `/home/xu/ros2_ws/lidar_support/mapping/`。
- 本次验证局部地图生成、保存和显示成功；尚未验证小车完整行走、闭环精度或自主导航。
- 测试期间树莓派发生过重启；恢复后已验证 GPIO14/15 UART 的开机配置生效，雷达驱动恢复正常测距。
- 桌面曾显示欠压提示，`vcgencmd get_throttled` 返回 `0x50000`，记录了历史欠压与降频，采样时当前低位状态为零。应检查树莓派供电稳定性。

建图进程、RViz2 与雷达驱动在交付时保持运行。没有配置建图开机自启。
