/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "chassis_link.h"
#include "servo_output.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//LED闪烁计数值
int Count=0;
//ADC读取值
u16 adc_val;
//电源电压值（放大了100倍）
u32 voltage;
//小车当前状态
u8 Car_Mode=0;
//小车轴距
float Axle_spacing = axlespacing;
//小车轮距
float Wheel_spacing = wheelspacing;
//PID parameters of Speed control
//p普通模式下的电机PID控制参数
float Velocity_KP=100,Velocity_KI=60;
//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;
//遥控控制的速度
float RC_Velocity=350;
//用于手机app的方向变量，顺时针一圈共8个方向，数值是1--8,停止时数值为0
u8 Flag_Direction;
//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send;
//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0;
//ROS发送过来的通信协议帧
u8 Receive_Data[12];

pIMUInterface_t imu = &UserICM20948;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	delay_init(72);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	ServoOutput_Init();
	ChassisLink_Init();
	HAL_ADCEx_Calibration_Start(&hadc1);
	OLED_Init();
	IMU_InitChecked();
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//开启定时器8通道1的PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//开启定时器8通道1的PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//开启定时器8通道1的PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//开启定时器8通道1的PWM输出
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2的编码器模式
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//开启定时器3的编码器模式
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4的编码器模式
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);//开启定时器5的编码器模式
	Motor_Init();
  /* USER CODE END 2 */
	
	/** @brief 启动 100 Hz 闭环，上电禁能，仅接受协议握手后的速度命令。 */
	HAL_TIM_Base_Start_IT(&htim6);

	/* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
	Battery_Update();
	ChassisLink_Poll();
	Show();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
