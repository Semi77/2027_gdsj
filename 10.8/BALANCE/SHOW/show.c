#include "show.h"

// Display a signed value to 0.1, clamped to the two-digit OLED field.
static void Show_Heading_Value(u8 y, const u8 *label, float value)
{
	int tenths;

	if (value > 99.9f) value = 99.9f;
	if (value < -99.9f) value = -99.9f;
	tenths = (int)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
	OLED_ShowString(74, y, label);
	OLED_ShowChar(90, y, tenths < 0 ? '-' : '+', 12, 1);
	if (tenths < 0) tenths = -tenths;
	OLED_ShowNumber(98, y, (u32)(tenths / 100), 1, 12);
	OLED_ShowNumber(104, y, (u32)((tenths / 10) % 10), 1, 12);
	OLED_ShowString(110, y, ".");
	OLED_ShowNumber(118, y, (u32)(tenths % 10), 1, 12);
}

// The internal heading is unwrapped. Show it modulo 360 degrees; use
// integer degrees above 99.9 so the value does not silently saturate.
static void Show_Heading_Angle(float angle_deg)
{
	int whole_degrees;
	while (angle_deg > 180.0f) angle_deg -= 360.0f;
	while (angle_deg < -180.0f) angle_deg += 360.0f;
	if (angle_deg > -99.95f && angle_deg < 99.95f)
	{
		Show_Heading_Value(10, "H:", angle_deg);
		return;
	}

	OLED_ShowString(74, 10, "H:");
	OLED_ShowChar(90, 10, angle_deg < 0.0f ? '-' : '+', 12, 1);
	if (angle_deg < 0.0f) angle_deg = -angle_deg;
	whole_degrees = (int)(angle_deg + 0.5f);
	OLED_ShowNumber(98, 10, (u32)(whole_degrees / 100), 1, 12);
	OLED_ShowNumber(104, 10, (u32)((whole_degrees / 10) % 10), 1, 12);
	OLED_ShowNumber(110, 10, (u32)(whole_degrees % 10), 1, 12);
}

static void Show_Lateral_Error(float error_mm)
{
	int millimetres;
	if (error_mm > 999.0f) error_mm = 999.0f;
	if (error_mm < -999.0f) error_mm = -999.0f;
	millimetres = (int)(error_mm + (error_mm >= 0.0f ? 0.5f : -0.5f));
	OLED_ShowString(74, 40, "Y:");
	OLED_ShowChar(90, 40, millimetres < 0 ? '-' : '+', 12, 1);
	if (millimetres < 0) millimetres = -millimetres;
	OLED_ShowNumber(98, 40, (u32)(millimetres / 100), 1, 12);
	OLED_ShowNumber(104, 40, (u32)((millimetres / 10) % 10), 1, 12);
	OLED_ShowNumber(110, 40, (u32)(millimetres % 10), 1, 12);
}

/** @brief 每 100 ms 更新轮速及状态显示，避免阻塞主循环持续刷新。 @retval 无。 */
void Show(void)
{
    static uint32_t last_tick;
    uint32_t tick = HAL_GetTick();
    if ((uint32_t)(tick - last_tick) < 100U)
        return;
    last_tick = tick;
	memset(OLED_GRAM,0, 128*8*sizeof(u8));
	/*************************第一行************************/
	OLED_ShowString(00, 00, "4WD");
	
	if (IMU_IsInitialized()) OLED_ShowString(80, 00, "IMU");
    else OLED_ShowString(80, 00, "ERR");
	
	/*************************第二行************************/
	OLED_ShowString(00, 10, "A:");
	if(MOTOR_A.Encoder>0)	OLED_ShowString(18, 10, "+"),
												OLED_ShowNumber(30, 10, MOTOR_A.Encoder*1000, 5, 12);
	else									OLED_ShowString(18, 10, "-"),
												OLED_ShowNumber(30, 10, -MOTOR_A.Encoder*1000, 5, 12);
	
	Show_Heading_Angle(Heading_Angle_Deg);
	/*************************第三行************************/
	OLED_ShowString(00, 20, "B:");
	if(MOTOR_B.Encoder>0)	OLED_ShowString(18, 20, "+"),
												OLED_ShowNumber(30, 20, MOTOR_B.Encoder*1000, 5, 12);
	else									OLED_ShowString(18, 20, "-"),
												OLED_ShowNumber(30, 20, -MOTOR_B.Encoder*1000, 5, 12);
	
	Show_Heading_Value(20, "G:", Heading_Gyro_Deg_S);
	/*************************第四行************************/
	OLED_ShowString(00, 30, "C:");
	if(MOTOR_C.Encoder>0)	OLED_ShowString(18, 30, "+"),
												OLED_ShowNumber(30, 30, MOTOR_C.Encoder*1000, 5, 12);
	else									OLED_ShowString(18, 30, "-"),
												OLED_ShowNumber(30, 30, -MOTOR_C.Encoder*1000, 5, 12);
	
	Show_Heading_Value(30, "T:", Heading_Turn_Deg_S);
	/*************************第五行************************/
	OLED_ShowString(00, 40, "D:");
	if(MOTOR_D.Encoder>0)	OLED_ShowString(18, 40, "+"),
												OLED_ShowNumber(30, 40, MOTOR_D.Encoder*1000, 5, 12);
	else									OLED_ShowString(18, 40, "-"),
												OLED_ShowNumber(30, 40, -MOTOR_D.Encoder*1000, 5, 12);
	
	Show_Lateral_Error(Lateral_Error_Mm);
	
	/*************************第六行************************/
	if(Car_Mode==APP_Mode)							OLED_ShowString(00, 50, "APP   ");
	else if(Car_Mode==ROS_Mode)					OLED_ShowString(00, 50, "ROS   ");
	
	OLED_ShowString(54, 50, "Vol:");
	OLED_ShowNumber(84, 50, voltage/100, 2, 12);
	OLED_ShowString(96, 50, ".");
	OLED_ShowNumber(102, 50, voltage%100, 2, 12);
	OLED_ShowString(114, 50, "V");
		
	OLED_Refresh_Gram();
}
