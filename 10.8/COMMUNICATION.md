# STM32 四轮执行接口 wheels-v2

本工程的控制策略已迁移到车载树莓派。完整部署顺序与参数见 [树莓派控制说明](../robot_console/PI_CONTROL.md)，逐字段协议见 [当前通信协议](../robot_console/PROTOCOL.md)。

## 固定职责

- TIM6 以 100 Hz 采集编码器、更新板载 IMU，并执行四轮速度 PI 和 PWM 输出。
- USART3 接收 A/B/C/D 四轮目标速度；不再计算 yaw 纠偏或从 vx/wz 分配轮速。
- 上电禁止电机使能；需要新会话同步、零速使能握手和 IMU 就绪。
- 300 ms 有效命令超时、接收错误和缓冲溢出禁能；保留低于 10 V 停止 PWM 的保护。
- 反馈每 20 ms 尝试发送，发送忙时跳过，不阻塞控制。

## 接线与协议

STM32 PC10/TX → 树莓派 GPIO9/RX（物理21），PC11/RX ← GPIO8/TX（物理24），共地。115200/8N1，树莓派 `/dev/ttyAMA3`。

帧头 `AA55`、version=1、type、负载长度u16、sequence u16、tick u32、负载、CRC16/CCITT-FALSE，小端。CRC 覆盖 version 至负载结尾。

新控制 type=0x02，13字节负载 `<IB4h`：session、enable、A/B/C/D。每轮目标单位 mm/s，范围 -300..300；禁能帧全部为零。初次使能也必须全部为零。重复/倒序、范围错误、CRC错误或错误会话不会刷新超时。

反馈 type=0x81，46字节负载保持原布局，固件标识改为 **0xC202**（wheels-v2）。包括使能/IMU/超时标志、会话、确认序号、四轮速度与目标、Roll/Pitch/Yaw、陀螺仪/加速度计原始值、电压。

旧 type=0x01 vx/wz 不再接受。新树莓派软件识别旧0xC201反馈，但对旧固件只允许监测及禁能。必须配套使用新软件和新固件。

## 源码入口

- `Core/Src/chassis_link.c`：串口帧、会话、四轮目标与超时。
- `Core/Inc/chassis_link.h`：执行接口声明。
- `BALANCE/CONCTRL/conctrl.c`：四轮速度 PI、传感器与 PWM 执行。
- `SYSTEM/delay/delay.c`：使用 DWT 计时，保留 HAL SysTick 正常运行；不要换回关闭 SysTick 的旧延时实现。
- `../robot_console/pi/heading_control.py`：已迁到树莓派的 yaw 外环及转向分配。

本次不编译或烧录；outcome 和 dist 中已有 HEX 均不是此次变更的构建产物。已做源码结构、协议字段和构建产物未变检查，新的 MCU 接口尚需用户编译并烧录后验证。

此次迁移前备份：`../output/pi_control_20261028/before_migration.zip`。

## 2026-10-29 执行接口完善

- 编码器改为读取自由运行定时器差值，不再读后清零；100Hz同时累计四轮uint32计数，停车和禁能仍计数。Control_GetEncoderSnapshot原子取得采样毫秒与四轮累计值。
- 新增0x82扩展反馈（32字节负载），提供累计计数、采样时间、会话、能力、故障和舵机输出确认。0x81原反馈保持不变。
- 新增Core/Src/servo_output.c，已加入Keil工程；main仅调用初始化。PA8/TIM1_CH1为50Hz、1μs分辨率，默认0脉宽。
- 新增0x03舵机指令，通道0、脉宽0或500..2500μs。通信失联保持最后脉宽；明确关闭或MCU复位才关闭。舵机指令不刷新车轮超时。
- 欠压时撤销车轮使能并清除PI，恢复供电后需重新使能。接收错误历史与当前故障位通过0x82上报。
- TIM1由servo_output.c独立初始化；使用CubeMX重新生成代码时必须保留该用户模块和main用户区调用，不要重复配置TIM1。TIM8车轮PWM不变。

硬件脉宽范围不是具体舵机的机械安全范围；具体型号未确认，树莓派servo_allowed默认false。新功能须用户编译烧录后验证；本次没有执行编译或烧录。参数、ROS入口及使用限制见robot_console/PI_CONTROL.md。
