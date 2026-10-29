#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

#include <stdint.h>

/** @brief 初始化 PA8/TIM1_CH1，50Hz 且默认无高电平脉冲。 @retval 无。 */
void ServoOutput_Init(void);
/** @brief 设置通用脉宽。 @param pulse_us 0 关闭，500..2500 为微秒脉宽。 @retval 1 成功，0 超限。 */
uint8_t ServoOutput_Set(uint16_t pulse_us);
/** @brief 获取当前输出脉宽，0 表示关闭，不代表实际舵机角度。 @retval 微秒脉宽。 */
uint16_t ServoOutput_Get(void);

#endif
