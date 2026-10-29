#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"


#define STBY	PCout(2)

#define CIN2	PDout(2)	
#define CIN1	PCout(12)

#define DIN2	PBout(4)
#define DIN1	PBout(5)

#define AIN2	PCout(13)	
#define AIN1	PCout(14)

#define BIN2	PBout(8)
#define BIN1	PBout(9)

//#define BIN1	PCout(12)
//#define BIN2	PDout(2)	

//#define AIN1	PBout(5)
//#define AIN2	PBout(4)

//#define DIN1	PCout(14)	
//#define DIN2	PCout(13)

//#define CIN1	PBout(9)
//#define CIN2	PBout(8)

#define		PWMA		TIM8->CCR3
#define 	PWMB		TIM8->CCR4
#define 	PWMC		TIM8->CCR1
#define		PWMD		TIM8->CCR2
void Motor_Init(void);

#endif
