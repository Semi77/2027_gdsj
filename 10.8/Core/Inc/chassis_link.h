#ifndef CHASSIS_LINK_H
#define CHASSIS_LINK_H

#include <stdint.h>

/** @brief 初始化 USART3 协议、接收缓冲及上电禁能状态。 @retval 无。 */
void ChassisLink_Init(void);
/** @brief 在 USART3 中断缓存接收字节。 @param byte 收到的原始字节。 @retval 无。 */
void ChassisLink_RxByte(uint8_t byte);
/** @brief 串口接收错误时立即禁能并请求清空缓冲。 @retval 无。 */
void ChassisLink_RxError(void);
/** @brief 主循环解析命令并非阻塞发送 50 Hz 反馈。 @retval 无。 */
void ChassisLink_Poll(void);
/**
  * @brief 在 100 Hz 控制中断检查就绪状态和 300 ms 命令超时。
  * @param ready 板载 IMU 初始化及静止校准是否完成。
  * @param wheels 输出 A/B/C/D 四轮目标速度，单位 m/s，数组长度为 4。
  * @retval 1 允许电机闭环，0 保持禁能。
  */
uint8_t ChassisLink_Control(uint8_t ready, float wheels[4]);
/** @brief 查询是否允许速度环积分。 @retval 1 已使能，0 已禁能。 */
uint8_t ChassisLink_IsArmed(void);

#endif
