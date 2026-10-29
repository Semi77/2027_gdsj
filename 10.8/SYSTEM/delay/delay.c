#include "delay.h"

/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-01-04

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-01-04

All rights reserved
***********************************************/

static uint32_t cycles_per_us = 1U;

/**
  * @brief 启用 Cortex-M3 周期计数器用于延时，保留 HAL 管理的 SysTick 不变。
  * @param SYSCLK 当前内核时钟，单位 MHz；传入 0 时使用 SystemCoreClock。
  * @retval 无。
  */
void delay_init(u8 SYSCLK)
{
    cycles_per_us = SYSCLK ? (uint32_t)SYSCLK : SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
        cycles_per_us = 1U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
}

/**
  * @brief 按 DWT 周期计数忙等待，分段计时避免长延时乘法溢出。
  * @param nus 延时时间，单位微秒；0 表示立即返回。
  * @retval 无。
  */
void delay_us(u32 nus)
{
    uint32_t chunk, ticks, start;
    while (nus != 0U)
    {
        chunk = (nus > 1000U) ? 1000U : nus;
        ticks = chunk * cycles_per_us;
        start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
        nus -= chunk;
    }
}

/**
  * @brief 使用微秒延时完成毫秒等待，不关闭或重配置 SysTick。
  * @param nms 延时时间，单位毫秒；0 表示立即返回。
  * @retval 无。
  */
void delay_ms(u16 nms)
{
    while (nms != 0U)
    {
        delay_us(1000U);
        --nms;
    }
}
