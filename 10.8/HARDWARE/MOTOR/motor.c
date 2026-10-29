#include "motor.h"

//初始化直流减速电机控制正反转的引脚
/** @brief 初始化时保持驱动禁能，等待协议零速使能握手。 @retval 无。 */
void Motor_Init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	STBY = 0;
}




