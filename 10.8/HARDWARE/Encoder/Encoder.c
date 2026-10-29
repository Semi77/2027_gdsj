#include "Encoder.h"

//void Encoder_TIM3_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
//	
//	TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;
//	TIM_TimeBaseInitStructure.TIM_Period = 65535;
//	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
//	
//	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 10;
//	TIM_ICInit(TIM3, &TIM_ICInitStructure);
//	
//	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM3, 0);
//	TIM_Cmd(TIM3, ENABLE);	
//}

//void Encoder_TIM5_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
//	
//	TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;
//	TIM_TimeBaseInitStructure.TIM_Period = 65535;
//	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
//	
//	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 10;
//	TIM_ICInit(TIM5, &TIM_ICInitStructure);
//	
//	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
//	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM5, 0);
//	TIM_Cmd(TIM5, ENABLE);	
//}

/**
  * @brief 读取自由运行计数器的有符号增量，不清零硬件计数器以避免读写间漏计数。
  * @param TIMX 编码器定时器编号，2..5；每路只能由100Hz采样函数读取。
  * @retval 本周期有符号计数；要求单周期绝对增量小于32768。
  */
int Read_Encoder(u8 TIMX)
{
    static uint16_t previous[4];
    uint16_t current, delta;
    switch (TIMX)
    {
        case 2: current = (uint16_t)TIM2->CNT; break;
        case 3: current = (uint16_t)TIM3->CNT; break;
        case 4: current = (uint16_t)TIM4->CNT; break;
        case 5: current = (uint16_t)TIM5->CNT; break;
        default: return 0;
    }
    delta = (uint16_t)(current - previous[TIMX - 2U]);
    previous[TIMX - 2U] = current;
    return delta < 32768U ? (int)delta : (int)delta - 65536;
}

void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//    ж 
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//    ж? ?λ 	    
}

void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//    ж 
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//    ж? ?λ 	    
}

void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//    ж 
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//    ж? ?λ 	    
}

void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001)//    ж 
	{    				   				     	    	
	}				   
	TIM5->SR&=~(1<<0);//    ж? ?λ 	    
}
