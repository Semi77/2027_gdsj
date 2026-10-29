#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
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
/** @brief 初始化 DWT 延时，保留 HAL SysTick。 @param SYSCLK 内核时钟 MHz。 @retval 无。 */
void delay_init(u8 SYSCLK);
/** @brief 毫秒忙等待。 @param nms 毫秒数。 @retval 无。 */
void delay_ms(u16 nms);
/** @brief 微秒忙等待。 @param nus 微秒数。 @retval 无。 */
void delay_us(u32 nus);

#endif





























