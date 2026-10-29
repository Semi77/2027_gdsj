# PA8舵机角度桥接

原GPIO12舵机程序已迁移为STM32 PA8后端。节点仍为`/servo_pwm_controller`，参数仍为`angle_deg`，输入话题仍为`/servo/angle_cmd`（`std_msgs/Float64`）。角度范围0–270度，线性映射500–2500微秒，周期20毫秒。微秒指令按整数发送，量化精度1微秒。

链路：角度节点 → `/servo/command` → `/chassis_controller` → UART3 → STM32 PA8/TIM1_CH1。角度节点不打开串口、不访问树莓派PWM。已有PA8固件无需因此重新编译或烧录。

原`/workspace/ros2_ws/.servo_pwm_state`保存文件不变，已保存136度。节点启动后等待新鲜底盘反馈，再恢复该目标；收到目标脉宽反馈后停止重发。反馈是PWM比较值，不能证明舵机到达实际角度。新指令若接口未就绪则拒绝并保留原角度，发送后2秒未确认则停止重发；重新提交角度或重启桥接节点可以再次请求。

树莓派原PWM0已经关闭，`ros2-servo`容器保留原开机重启策略。原桌面交互脚本、`servo-angle`和`servo-angle-set`都改为ROS接口，避免重新开启GPIO12。停止角度节点不会关闭STM32 PWM，失联保持最后输出。

使用前断电，将舵机信号线接到C12H H3的PA8信号脚，确认供电及共地。上位机连接并打开底盘串口后，启动目标会恢复；无需使能车轮。上位机直接输入脉宽和ROS角度指令共用PA8，最后被接收的指令生效；上位机直接调脉宽不会更新角度节点保存的目标。

配置已在树莓派开启`servo_allowed=true`、`servo_min_us=500`、`servo_max_us=2500`，保留用户要求的原程序映射；此映射来自原程序，未新增机械角度校准。

保留的命令：

```bash
/home/xu/servo-angle status
/home/xu/servo-angle-set 136
/home/xu/servo-angle set 136
```

2026-10-29验证：3项离线测试通过。跨容器通信使用随节点配置的Fast DDS UDP传输，避免共享内存隔离导致消息无法到达。配置方式依据[Fast DDS传输配置文档](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/interfaces.html)。12秒实机测试收到600帧反馈、CRC错误0、车轮禁能且速度为0；STM32确认PA8输出1507微秒（目标136度），舵机节点记录确认成功，故障位0。该确认只证明PWM指令生效，实际机械角度仍需观察。测试后底盘串口已释放，PA8保持最后输出。修改前备份位于本地`output/servo_pa8_20261029`和树莓派原文件旁的`.before_pa8_20261029`文件。
