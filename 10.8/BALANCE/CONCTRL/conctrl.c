#include "conctrl.h"
#include "chassis_link.h"

Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
Encoder OriginalEncoder;
Smooth_Control smooth_control;
volatile uint32_t Vofa_Control_Ticks = 0;
static volatile uint32_t encoder_totals[4];
static volatile uint32_t encoder_sample_ms;

#define CONTROL_DT_S                 0.01f
#define HEADING_CALIBRATION_SAMPLES  200U
#define GYRO_LSB_TO_RAD_S            (0.06103515625f * 0.01745329252f)
#define RAD_TO_DEG                   57.2957795f

static uint16_t heading_calibration_count;
static int32_t heading_gyro_sum;
static float heading_gyro_bias;
static float heading_angle;
static float heading_rate;
volatile float Heading_Angle_Deg = 0.0f;
volatile float Heading_Gyro_Deg_S = 0.0f;
volatile float Heading_Turn_Deg_S = 0.0f;
volatile float Lateral_Error_Mm = 0.0f;

/** @brief 累计静止样本校准陀螺零偏。 @retval 1 校准完成，0 正在校准。 */
static uint8_t Heading_Gyro_Ready(void)
{
	if (heading_calibration_count < HEADING_CALIBRATION_SAMPLES)
	{
		heading_gyro_sum += gyro[2];
		heading_calibration_count++;
		if (heading_calibration_count == HEADING_CALIBRATION_SAMPLES)
		{
			heading_gyro_bias = (float)heading_gyro_sum /
				(float)HEADING_CALIBRATION_SAMPLES;
		}
		return 0U;
	}
	return 1U;
}

/** @brief 更新本地相对航向与 OLED 角速度，周期为 10 ms。 @retval 无。 */
static void Heading_Update(void)
{
	heading_rate = ((float)gyro[2] - heading_gyro_bias) *
		GYRO_LSB_TO_RAD_S;
	heading_angle += heading_rate * CONTROL_DT_S;
	Heading_Angle_Deg = heading_angle * RAD_TO_DEG;
	Heading_Gyro_Deg_S = heading_rate * RAD_TO_DEG;
}

/**
  * @brief 以 100 Hz 执行编码器采样、协议超时检查及四轮速度闭环。
  * @param htim 触发回调的定时器句柄，仅处理 TIM6。
  * @retval 无。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float wheel_targets[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t ready = 0U;
    if (htim->Instance != TIM6)
        return;
    if (++Count >= 20)
    {
        Count = 0;
        LED = ~LED;
    }
    if (IMU_IsInitialized())
    {
        Get_Angle();
        ready = Heading_Gyro_Ready();
        if (ready) Heading_Update();
    }
    Get_Velocity_Form_Encoder();
    if (ChassisLink_Control(ready, wheel_targets) == 0U)
    {
        MOTOR_A.Target = MOTOR_B.Target = MOTOR_C.Target = MOTOR_D.Target = 0.0f;
        MOTOR_A.Motor_Pwm = Incremental_PI_A(0.0f, 0.0f);
        MOTOR_B.Motor_Pwm = Incremental_PI_B(0.0f, 0.0f);
        MOTOR_C.Motor_Pwm = Incremental_PI_C(0.0f, 0.0f);
        MOTOR_D.Motor_Pwm = Incremental_PI_D(0.0f, 0.0f);
        Move_X = Move_Y = Move_Z = Heading_Turn_Deg_S = 0.0f;
        Set_Pwm(0, 0, 0, 0);
        Vofa_Control_Ticks++;
        return;
    }
    /* 树莓派已完成航向纠偏与轮速分配，STM32 直接执行四轮目标。 */
    MOTOR_A.Target = wheel_targets[0];
    MOTOR_B.Target = wheel_targets[1];
    MOTOR_C.Target = wheel_targets[2];
    MOTOR_D.Target = wheel_targets[3];
    if (Turn_Off(voltage) == 0U)
    {
        MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
        MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
        MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
        MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
    }
    else
        MOTOR_A.Motor_Pwm = MOTOR_B.Motor_Pwm = MOTOR_C.Motor_Pwm = MOTOR_D.Motor_Pwm = 0.0f;
    Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm);
    Vofa_Control_Ticks++;
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			AIN1=1,AIN2=0,PWMA=-motor_a;
	else 	            AIN1=0,AIN2=1,PWMA=motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			BIN1=1,BIN2=0,PWMB=-motor_b;
	else 	            BIN1=0,BIN2=1,PWMB=motor_b;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			CIN1=1,CIN2=0,PWMC=-motor_c;
	else 	            CIN1=0,CIN2=1,PWMC=motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			DIN1=1,DIN2=0,PWMD=-motor_d;
	else 	            DIN1=0,DIN2=1,PWMD=motor_d;
}





/** @brief 原子取得累计计数和同周期采样时间，计数按32位模运算回绕。 @param totals 长度4数组。 @param sample_ms 采样毫秒。 @retval 无。 */
void Control_GetEncoderSnapshot(uint32_t totals[4], uint32_t *sample_ms)
{
    uint32_t irq = __get_PRIMASK();
    uint8_t i;
    __disable_irq();
    for (i = 0U; i < 4U; ++i) totals[i] = encoder_totals[i];
    *sample_ms = encoder_sample_ms;
    __set_PRIMASK(irq);
}

/** @brief 单次读取四轮编码器增量，同时用于速度闭环和累计反馈。 @retval 无。 */
void Get_Velocity_Form_Encoder(void)
{
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	int Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
	
	OriginalEncoder.A = Read_Encoder(5);
	OriginalEncoder.B = Read_Encoder(3);
	OriginalEncoder.C = Read_Encoder(4);
	OriginalEncoder.D = Read_Encoder(2);
	/* 在唯一的 100Hz 采样位置累计，禁能和停车期间也保留计数。 */
	encoder_totals[0] += (uint32_t)OriginalEncoder.A;
	encoder_totals[1] += (uint32_t)OriginalEncoder.B;
	encoder_totals[2] += (uint32_t)OriginalEncoder.C;
	encoder_totals[3] += (uint32_t)OriginalEncoder.D;
	encoder_sample_ms = HAL_GetTick();
	
	Encoder_A_pr = OriginalEncoder.A;
	Encoder_B_pr = OriginalEncoder.B;
	Encoder_C_pr = OriginalEncoder.C;
	Encoder_D_pr = OriginalEncoder.D;
	
	//The encoder converts the raw data to wheel speed in mm/s
	//编码器原始数据转换为车轮速度，单位mm/s
	//轮周长及每圈计数使用 sys.h 中当前配置，速度单位为 m/s。
	MOTOR_A.Encoder = Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_B.Encoder = Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_C.Encoder = Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_D.Encoder = Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	
}

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
     /** @brief 禁能时清除 PI 积分及上次误差，重新使能不继承旧 PWM。 */
     if (ChassisLink_IsArmed() == 0U)
     {
         Bias = Pwm = Last_bias = 0.0f;
         return 0;
     }
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
     /** @brief 禁能时清除 PI 积分及上次误差，重新使能不继承旧 PWM。 */
     if (ChassisLink_IsArmed() == 0U)
     {
         Bias = Pwm = Last_bias = 0.0f;
         return 0;
     }
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_C (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
     /** @brief 禁能时清除 PI 积分及上次误差，重新使能不继承旧 PWM。 */
     if (ChassisLink_IsArmed() == 0U)
     {
         Bias = Pwm = Last_bias = 0.0f;
         return 0;
     }
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_D (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
     /** @brief 禁能时清除 PI 积分及上次误差，重新使能不继承旧 PWM。 */
     if (ChassisLink_IsArmed() == 0U)
     {
         Bias = Pwm = Last_bias = 0.0f;
         return 0;
     }
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}

/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.1;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}

/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1000)  				temp=1; 
			else										temp=0;
			return temp;			
}

/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾
	
	Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000;
	Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder-MOTOR_D.Encoder)/4)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Axle_spacing+Wheel_spacing))*1000; 
	
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2]; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //The Y-axis is converted to the ros coordinate Y axis //角速度计Y轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Gyroscope.Z_data= gyro[2]; //The Z-axis is converted to the ros coordinate Z axis //角速度计Z轴转换到ROS坐标Z轴
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = voltage*10; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
	Send_Data.buffer[1]=0; 				//This is reserved, can be expanded  //该为做保留，可自行拓展
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾
}

/**************************************************************************
Function: Get Angle function
Input   : none
Output  : none
函数功能：获取角度信息
入口参数：无
返回  值：无
**************************************************************************/
void Get_Angle(void)
{
	imu->Update_9axisVal(&axis_9Val);           //陀螺仪数据更新
	imu->UpdateAttitude(axis_9Val,&AttitudeVal);//更新姿态角
	//*57.2958是将弧度转化为角度
	Roll  =( AttitudeVal.roll*57.2958);
	Pitch =( AttitudeVal.pitch*57.2958);
	Yaw = AttitudeVal.yaw*57.2958;
}


