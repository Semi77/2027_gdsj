# 树莓派雷达驱动验证记录

验证完成时间：2026-10-25，Asia/Shanghai。

## 实机与配置

- 树莓派：Raspberry Pi 5 Model B Rev 1.1。
- 雷达标签：DELTA2C Pro-D-V001；TX 接 GPIO15（物理 10 脚）。
- UART：`/dev/ttyAMA0`，115200 8N1，GPIO14/15 为 TXD0/RXD0。
- `/boot/firmware/config.txt` 已添加 `[pi5] dtoverlay=uart0-pi5`，保留原有 GPIO12 PWM。
- 备份：`/boot/firmware/config.txt.bak_lidar_20261024_234556`。
- 当前动态 overlay 已生效；未执行重启，因此下次启动加载配置尚未通过重启实测。
- `ros2-shell` 保留用户 1000、GPIO 主组及宿主机附加组，验证包含 dialout 20 和 gpio 986。
- 手动 ROS 包：`/home/xu/ros2_ws/src/delta2c_lidar`；没有配置节点开机自启。

## 同一采样的型号比较

原始采样：`/home/xu/ros2_ws/lidar_captures/gpio15_20261024_234556.bin`，12 秒、17512 字节。

使用上游 lds2d 的 26 个不同型号解码器离线回放同一采样，所有解码器都没有输出测距点，且没有向硬件发送指令。完整结果在 `model_comparison.json`。

严格 2C 协议解析得到 1592 帧转速报文，版本 0x13，转速 8.15–8.35 Hz，测距帧 0，校验错误 0。此采样本身没有测距数据，不能通过替换型号解码器恢复距离。

注意：被测试的上游 2C 解码器将长度字段下限设为 10，而实机 0xAE 报文的长度字段是 9、总长 11 字节，所以它不输出这些转速帧。本驱动按实际协议分别验证转速和测距帧长度，正确接受 0xAE。

实际运行 ROS 驱动后的独立诊断快照：17006 字节、1546 帧转速、0 帧测距、0 校验错误，转速 8.75 Hz。`/scan` 类型为 LaserScan、Best Effort，有发布端但没有实际扫描消息。固定 3.3 V 电机供电下，这次实测尚不具备连续测距条件；驱动没有电机调速接口。

## 软件验证

- 7 项协议测试通过：11 字节转速帧、0x11/0x13、四分之一毫米精度、逐字节接收、噪声与坏校验恢复、角度回绕、无回波过滤、帧长度、首次半圈与断流组帧。
- 3 份上游真实采样与 lds2d 交叉验证通过，测距帧分别为 618、447、205，总计 1270；校验错误为 0。
- Humble 伪终端 ROS 集成测试通过：无数据 → 仅转速 → 正常扫描；收到 11 帧 LaserScan，每帧 360 个槽、240 个有效距离，距离为 1.00025 米。
- 实机 ROS 串口接收和 `/diagnostics` 状态正确；`rosdep check --from-paths src --ignore-src` 全部满足。
- 尚未完成实机持续 `/scan`、距离尺量和方向零点标定，原因是当前仅输出转速帧。
- 实机测试后已停止雷达测试节点，避免占用用户手动启动所需的串口；既有舵机节点保持运行。

## 使用的参考版本

- hapynfdj/delta2c-lidar：`2ef20a34e1fd4b461dd8d467ee41c08a0249dd0a`
- kaiaai/lds2d：`58a1e90a67e16c8d56acccb93f566fd9a928de70`
- kaiaai/LDS：`6e17803760908b3898d3b7f3f912e8c4f8b7bafd`

本地验证脚本：`tools/compare_models.py`、`tools/validate_captures.py`；包内测试为 `delta2c_lidar/test/test_protocol.py`、`delta2c_lidar/test/verify_ros.py`。
