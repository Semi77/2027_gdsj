#include "servo_output.h"
#include "main.h"

static TIM_HandleTypeDef servo_timer;
static uint16_t servo_pulse;

/** @brief 使用独立 TIM1 硬件产生舵机脉冲，初始化时不驱动舵机转动。 @retval 无。 */
void ServoOutput_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef channel = {0};
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) timer_clock *= 2U;
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    servo_timer.Instance = TIM1;
    servo_timer.Init.Prescaler = timer_clock / 1000000U - 1U;
    servo_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo_timer.Init.Period = 19999U;
    servo_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    servo_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&servo_timer) != HAL_OK) Error_Handler();
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    channel.OCIdleState = TIM_OCIDLESTATE_RESET;
    channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&servo_timer, &channel, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    servo_pulse = 0U;
    if (HAL_TIM_PWM_Start(&servo_timer, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

/** @brief 只更新硬件比较值，通信失联时持续保持最后脉宽。 @param pulse_us 0 或500..2500微秒。 @retval 是否有效。 */
uint8_t ServoOutput_Set(uint16_t pulse_us)
{
    if (pulse_us != 0U && (pulse_us < 500U || pulse_us > 2500U)) return 0U;
    __HAL_TIM_SET_COMPARE(&servo_timer, TIM_CHANNEL_1, pulse_us);
    servo_pulse = pulse_us;
    return 1U;
}

/** @brief 返回已提交给定时器的脉宽，不提供舵机位置回读。 @retval 微秒脉宽。 */
uint16_t ServoOutput_Get(void)
{
    return servo_pulse;
}
