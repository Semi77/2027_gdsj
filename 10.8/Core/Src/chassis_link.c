#include "chassis_link.h"
#include "sys.h"
#include "usart.h"
#include "servo_output.h"
#include <string.h>

#define LINK_RING_SIZE       256U
#define LINK_TIMEOUT_MS      300U
#define LINK_MAX_PAYLOAD     80U
#define LINK_HEADER_SIZE     12U
#define LINK_COMMAND_SIZE    13U
#define LINK_FEEDBACK_SIZE   46U
#define LINK_EXTRA_SIZE      32U
#define LINK_STATUS_FRAME    (LINK_HEADER_SIZE + LINK_FEEDBACK_SIZE + 2U)

static volatile uint8_t rx_ring[LINK_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static uint8_t frame[LINK_HEADER_SIZE + LINK_MAX_PAYLOAD + 2U];
static uint16_t frame_count;
static volatile uint32_t session_id, last_valid_tick;
static volatile uint16_t last_sequence;
static volatile uint8_t session_seen, armed, timeout_fault, imu_ready;
static volatile int16_t command_wheels[4];
static uint8_t tx_frame[LINK_STATUS_FRAME + LINK_HEADER_SIZE + LINK_EXTRA_SIZE + 2U];
static uint16_t servo_sequence;
static uint8_t servo_seen;
static volatile uint8_t receive_error_seen;
static uint16_t tx_sequence;
static uint32_t last_feedback_tick;

/** @brief 读取小端字段。 @param p 字段起始地址。 @retval 无符号 16 位值。 */
static uint16_t Read16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/** @brief 读取小端字段。 @param p 字段起始地址。 @retval 无符号 32 位值。 */
static uint32_t Read32(const uint8_t *p)
{
    return (uint32_t)Read16(p) | ((uint32_t)Read16(p + 2) << 16);
}

/** @brief 写入小端字段。 @param p 目标地址。 @param value 16 位值。 @retval 无。 */
static void Write16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

/** @brief 写入小端字段。 @param p 目标地址。 @param value 32 位值。 @retval 无。 */
static void Write32(uint8_t *p, uint32_t value)
{
    Write16(p, (uint16_t)value);
    Write16(p + 2, (uint16_t)(value >> 16));
}

/** @brief 计算 CCITT-FALSE 校验。 @param data 输入字节。 @param length 长度。 @retval CRC16。 */
static uint16_t Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU, i;
    uint8_t bit;
    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

/** @brief 初始化禁能状态，并在所有外设初始化后打开 USART3 接收。 @retval 无。 */
void ChassisLink_Init(void)
{
    rx_head = rx_tail = frame_count = 0U;
    rx_fault = session_seen = armed = timeout_fault = imu_ready = 0U;
    servo_sequence = 0U;
    servo_seen = receive_error_seen = 0U;
    session_id = last_valid_tick = 0U;
    last_sequence = tx_sequence = 0U;
    memset((void *)command_wheels, 0, sizeof(command_wheels));
    Move_X = Move_Y = Move_Z = 0.0f;
    Car_Mode = ROS_Mode;
    last_feedback_tick = HAL_GetTick();
    STBY = 0;
    Set_Pwm(0, 0, 0, 0);
    (void)HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
}

/** @brief 接收中断只入队，溢出时禁能。 @param byte 原始接收字节。 @retval 无。 */
void ChassisLink_RxByte(uint8_t byte)
{
    uint16_t next = (rx_head + 1U) & (LINK_RING_SIZE - 1U);
    if (next == rx_tail)
    {
        ChassisLink_RxError();
        return;
    }
    rx_ring[rx_head] = byte;
    rx_head = next;
}

/** @brief 接收错误立即撤销驱动使能，主循环随后丢弃残帧。 @retval 无。 */
void ChassisLink_RxError(void)
{
    rx_fault = 1U;
    receive_error_seen = 1U;
    armed = 0U;
    STBY = 0;
}

/** @brief 原子提交已校验命令，非零速度只能在零速使能握手后接受。 @retval 无。 */
static void AcceptCommand(void)
{
    uint32_t received_session = Read32(frame + 12);
    uint16_t sequence = Read16(frame + 6);
    uint16_t delta;
    uint8_t enable = frame[16];
    int16_t wheels[4];
    uint8_t i, all_zero = 1U;
    uint32_t irq;
    if (received_session == 0U || enable > 1U)
        return;
    for (i = 0U; i < 4U; ++i)
    {
        wheels[i] = (int16_t)Read16(frame + 17 + 2U * i);
        if (wheels[i] < -300 || wheels[i] > 300)
            return;
        if (wheels[i] != 0) all_zero = 0U;
    }
    if (enable == 0U && !all_zero)
        return;
    irq = __get_PRIMASK();
    __disable_irq();
    delta = (uint16_t)(sequence - last_sequence);
    if (!rx_fault)
    {
        if (enable == 0U)
        {
            if (received_session != session_id) servo_seen = 0U;
            armed = 0U;
            memset((void *)command_wheels, 0, sizeof(command_wheels));
            session_id = received_session;
            session_seen = 1U;
            last_sequence = sequence;
            STBY = 0;
        }
        else if (session_seen && received_session == session_id &&
                 delta > 0U && delta < 0x8000U && imu_ready)
        {
            if (armed || all_zero)
            {
                for (i = 0U; i < 4U; ++i) command_wheels[i] = wheels[i];
                last_sequence = sequence;
                last_valid_tick = HAL_GetTick();
                timeout_fault = 0U;
                armed = 1U;
            }
        }
    }
    __set_PRIMASK(irq);
}

/** @brief 接收独立舵机命令，单独确认序号且绝不刷新车轮超时。 @retval 无。 */
static void AcceptServo(void)
{
    uint32_t received_session = Read32(frame + 12);
    uint16_t sequence = Read16(frame + 6);
    uint16_t delta = (uint16_t)(sequence - servo_sequence);
    uint8_t channel = frame[16], enable = frame[17];
    uint16_t pulse = Read16(frame + 18);
    if (rx_fault || !session_seen || received_session != session_id ||
        channel != 0U || enable > 1U || (enable == 0U && pulse != 0U) ||
        (enable == 1U && (pulse < 500U || pulse > 2500U))) return;
    if (servo_seen && (delta == 0U || delta >= 0x8000U)) return;
    if (ServoOutput_Set(pulse))
    {
        servo_sequence = sequence;
        servo_seen = 1U;
    }
}

/** @brief 按长度和 CRC 解析字节流，坏帧逐字节重新同步。 @param byte 输入字节。 @retval 无。 */
static void ParseByte(uint8_t byte)
{
    uint16_t total;
    if (frame_count >= sizeof(frame))
        frame_count = 0U;
    frame[frame_count++] = byte;
    while (frame_count >= 2U)
    {
        if (frame[0] != 0xAAU || frame[1] != 0x55U)
        {
            memmove(frame, frame + 1, --frame_count);
            continue;
        }
        if (frame_count < LINK_HEADER_SIZE)
            return;
        if (frame[2] != 1U || Read16(frame + 4) > LINK_MAX_PAYLOAD)
        {
            memmove(frame, frame + 1, --frame_count);
            continue;
        }
        total = LINK_HEADER_SIZE + Read16(frame + 4) + 2U;
        if (frame_count < total)
            return;
        if (Crc16(frame + 2, total - 4U) != Read16(frame + total - 2U))
        {
            memmove(frame, frame + 1, --frame_count);
            continue;
        }
        if (frame[3] == 2U && Read16(frame + 4) == LINK_COMMAND_SIZE)
            AcceptCommand();
        else if (frame[3] == 3U && Read16(frame + 4) == 8U)
            AcceptServo();
        frame_count -= total;
        memmove(frame, frame + total, frame_count);
    }
}

/** @brief 控制中断执行独立超时检查并输出 SI 速度，参数及返回值见头文件。 */
uint8_t ChassisLink_Control(uint8_t ready, float wheels[4])
{
    uint8_t i;
    imu_ready = ready;
    if (armed && (uint32_t)(HAL_GetTick() - last_valid_tick) >= LINK_TIMEOUT_MS)
    {
        armed = 0U;
        timeout_fault = 1U;
    }
    if (!ready || voltage < 1000U)
        armed = 0U;
    if (!armed)
    {
        memset((void *)command_wheels, 0, sizeof(command_wheels));
        for (i = 0U; i < 4U; ++i) wheels[i] = 0.0f;
        STBY = 0;
        return 0U;
    }
    for (i = 0U; i < 4U; ++i) wheels[i] = (float)command_wheels[i] * 0.001f;
    STBY = 1;
    return 1U;
}

/** @brief 返回协议使能状态，速度 PI 禁能时必须清除历史。 @retval 当前使能状态。 */
uint8_t ChassisLink_IsArmed(void)
{
    return armed;
}

/** @brief 打包一致的四轮及 IMU 快照，发送忙时跳过本周期。 @param tick 毫秒时间。 @retval 无。 */
static void SendFeedback(uint32_t tick)
{
    uint8_t *p = tx_frame + LINK_HEADER_SIZE;
    uint8_t *extra = tx_frame + LINK_STATUS_FRAME;
    uint8_t *e = extra + LINK_HEADER_SIZE;
    uint32_t totals[4], sample_ms;
    Motor_parameter motors[4];
    int16_t local_gyro[3], local_accel[3];
    float rpy[3];
    uint16_t flags;
    uint32_t irq;
    uint8_t i;
    if (huart3.gState != HAL_UART_STATE_READY)
        return;
    tx_frame[0] = 0xAAU;
    tx_frame[1] = 0x55U;
    tx_frame[2] = 1U;
    tx_frame[3] = 0x81U;
    Write16(tx_frame + 4, LINK_FEEDBACK_SIZE);
    Write16(tx_frame + 6, ++tx_sequence);
    Write32(tx_frame + 8, tick);
    irq = __get_PRIMASK();
    __disable_irq();
    motors[0] = MOTOR_A; motors[1] = MOTOR_B;
    motors[2] = MOTOR_C; motors[3] = MOTOR_D;
    flags = (uint16_t)(armed | (imu_ready << 1) | (timeout_fault << 2));
    Write16(p, 0xC202U);
    Write16(p + 2, flags);
    Write32(p + 4, session_id);
    Write16(p + 8, last_sequence);
    rpy[0] = Roll;
    rpy[1] = Pitch;
    rpy[2] = Yaw;
    for (i = 0U; i < 3U; ++i)
    {
        local_gyro[i] = gyro[i];
        local_accel[i] = accel[i];
    }
    Write16(p + 44, (uint16_t)voltage);
    Control_GetEncoderSnapshot(totals, &sample_ms);
    Write32(e, sample_ms);
    Write32(e + 4, session_id);
    for (i = 0U; i < 4U; ++i) Write32(e + 8 + 4U * i, totals[i]);
    Write16(e + 24, 7U); /* bit0 累计编码器，bit1 PA8舵机，bit2 故障反馈。 */
    Write16(e + 26, (uint16_t)((voltage < 1000U) | (receive_error_seen << 1) |
            (timeout_fault << 2) | ((!imu_ready) << 3)));
    Write16(e + 28, ServoOutput_Get());
    Write16(e + 30, servo_sequence);
    __set_PRIMASK(irq);
    for (i = 0U; i < 4U; ++i)
    {
        Write16(p + 10 + 2U * i, (uint16_t)(int16_t)(motors[i].Encoder * 1000.0f));
        Write16(p + 18 + 2U * i, (uint16_t)(int16_t)(motors[i].Target * 1000.0f));
    }
    for (i = 0U; i < 3U; ++i)
    {
        Write16(p + 26 + 2U * i, (uint16_t)(int16_t)(rpy[i] * 100.0f));
        Write16(p + 32 + 2U * i, (uint16_t)local_gyro[i]);
        Write16(p + 38 + 2U * i, (uint16_t)local_accel[i]);
    }
    Write16(tx_frame + LINK_STATUS_FRAME - 2U, Crc16(tx_frame + 2, LINK_STATUS_FRAME - 4U));
    extra[0] = 0xAAU;
    extra[1] = 0x55U;
    extra[2] = 1U;
    extra[3] = 0x82U;
    Write16(extra + 4, LINK_EXTRA_SIZE);
    Write16(extra + 6, tx_sequence);
    Write32(extra + 8, tick);
    Write16(extra + LINK_HEADER_SIZE + LINK_EXTRA_SIZE,
            Crc16(extra + 2, LINK_HEADER_SIZE + LINK_EXTRA_SIZE - 2U));
    (void)HAL_UART_Transmit_IT(&huart3, tx_frame, sizeof(tx_frame));
}

/** @brief 有界解析接收缓冲、恢复接收并尝试发送 50 Hz 最新反馈。 @retval 无。 */
void ChassisLink_Poll(void)
{
    uint16_t budget = 128U;
    uint8_t byte;
    uint32_t tick = HAL_GetTick();
    if (rx_fault)
    {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head;
        rx_fault = 0U;
        __set_PRIMASK(irq);
        frame_count = 0U;
    }
    if (huart3.RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
    while (rx_tail != rx_head && budget--)
    {
        byte = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) & (LINK_RING_SIZE - 1U);
        ParseByte(byte);
    }
    if ((uint32_t)(tick - last_feedback_tick) >= 20U)
    {
        last_feedback_tick = tick;
        SendFeedback(tick);
    }
}
