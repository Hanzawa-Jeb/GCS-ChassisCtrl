#include "emm42_can_driver.h"  // 包含自定义头文件
#include "string.h"            // 包含字符串处理函数

// 电机ID数组（CAN地址）
static const uint8_t motor_addresses[MOTOR_NUM] = {
        MOTOR_FL_ID,
        MOTOR_FR_ID,
        MOTOR_RL_ID,
        MOTOR_RR_ID
};

// 初始化底盘电机驱动（设置初始状态并配置CAN外设）
void Emm42_Init(Chassis_Control_t *chassis, FDCAN_HandleTypeDef *hfdcan, Checksum_Mode checksum_mode)
{
    // 保存CAN句柄和校验方式到结构体
    chassis->hfdcan = hfdcan;
    chassis->checksum_mode = checksum_mode;
    chassis->sync_flag = 0;  // 初始同步标志为0（未同步）
    chassis->comm_ok = 1;    // 初始通信状态为正常

    // 初始化所有电机数据
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        chassis->motors[i].address = motor_addresses[i];  // 设置电机地址
        chassis->motors[i].control_mode = CONTROL_MODE_VELOCITY; // 默认速度控制模式
        chassis->motors[i].is_enabled = 0;                 // 初始为未使能
        chassis->motors[i].error_code = 0;                 // 初始无错误

        // 速度模式参数初始化
        chassis->motors[i].target_velocity = 0;            // 目标速度为0
        chassis->motors[i].actual_velocity = 0;            // 实际速度为0
        chassis->motors[i].acceleration = 0;               // 加速度为0

        // 位置模式参数初始化
        chassis->motors[i].target_position = 0;            // 目标位置为0
        chassis->motors[i].actual_position = 0;            // 实际位置为0
        chassis->motors[i].pos_mode = POSITION_ABSOLUTE;   // 默认绝对位置模式

        // 状态标志初始化
        chassis->motors[i].is_moving = 0;                  // 初始为静止
        chassis->motors[i].position_reached = 0;           // 位置未到达
        chassis->motors[i].temperature = 0;                // 温度为0
    }

    // 配置FDCAN滤波器（设置接收哪些FDCAN消息）
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    // 应用FDCAN滤波器配置
    if (HAL_FDCAN_ConfigFilter(chassis->hfdcan, &can_filter) != HAL_OK)
    {
        Error_Handler();  // 如果配置失败，调用错误处理函数
    }

    // 启动FDCAN外设
    if (HAL_FDCAN_Start(chassis->hfdcan) != HAL_OK)
    {
        Error_Handler();  // 如果启动失败，调用错误处理函数
    }

    // 启动CAN接收中断（当FIFO0中有消息时触发中断）
    if (HAL_FDCAN_ActivateNotification(chassis->hfdcan, FDCAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();  // 如果启用中断失败，调用错误处理函数
    }
}

// 设置电机控制模式
void Emm42_SetControlMode(Chassis_Control_t *chassis, uint8_t motor_index, Control_Mode mode)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return;  // 参数检查
    }

    chassis->motors[motor_index].control_mode = mode;
}

// 计算校验和（根据配置的校验方式计算校验字节）
uint8_t Calculate_Checksum(Chassis_Control_t *chassis, uint8_t *data, uint8_t len)
{
    if (chassis == NULL || data == NULL) {
        return 0x6B;  // 参数检查，返回默认校验值
    }

    // 根据配置的校验方式选择计算方法
    switch (chassis->checksum_mode)
    {
        case CHECKSUM_FIXED:
            return 0x6B;  // 固定返回0x6B

        case CHECKSUM_XOR:
        {
            // 异或校验：对所有数据字节进行异或操作
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < len; i++)
            {
                checksum ^= data[i];  // 逐字节异或
            }
            return checksum;
        }

        case CHECKSUM_CRC8:
        {
            // 标准CRC8计算（多项式: x^8 + x^2 + x + 1, 0x07）
            uint8_t crc = 0;
            for (uint8_t i = 0; i < len; i++)
            {
                crc ^= data[i];  // 先与数据字节异或
                for (uint8_t j = 0; j < 8; j++)
                {
                    if (crc & 0x80) {
                        crc = (crc << 1) ^ 0x07;  // 如果最高位为1，异或多项式
                    } else {
                        crc <<= 1;  // 否则直接左移
                    }
                }
            }
            return crc;
        }

        default:
            return 0x6B;  // 默认使用固定0x6B校验
    }
}

// 发送FDCAN消息（封装HAL库的FDCAN发送函数）
HAL_StatusTypeDef Emm42_SendCANMessage(Chassis_Control_t *chassis, uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (chassis == NULL || chassis->hfdcan == NULL || data == NULL) {
        return HAL_ERROR;  // 参数检查
    }

    FDCAN_TxHeaderTypeDef TxHeader;  // FDCAN发送消息头结构

    // 配置FDCAN发送头
    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 8字节数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 添加消息到发送邮箱并请求发送
    return HAL_FDCAN_AddTxMessageToTxFifoQ(chassis->hfdcan, &TxHeader, data);
}

// 电机使能控制
HAL_StatusTypeDef Emm42_EnableMotor(Chassis_Control_t *chassis, uint8_t motor_index, uint8_t enable)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return HAL_ERROR;  // 参数检查
    }

    uint8_t data[5];
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_ENABLE_CONTROL;                   // 功能码（使能控制）
    data[2] = 0xAB;                                  // 固定值（协议要求）
    data[3] = enable ? 0x01 : 0x00;                  // 使能状态（1使能，0不使能）
    data[4] = 0x00;                                  // 多机同步标志（0不启用）

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 5);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[6];
    memcpy(send_data, data, 5);  // 复制前5个字节
    send_data[5] = checksum;     // 添加校验字节

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 6);

    if (status == HAL_OK) {
        chassis->motors[motor_index].is_enabled = enable;
    } else {
        chassis->comm_ok = 0;
    }

    // 更新本地使能状态
    if (enable) {
        chassis->motors[motor_index].is_enabled = 1;
    } else {
        chassis->motors[motor_index].is_enabled = 0;
    }

    return status;
}

// 设置单个电机速度（控制指定电机的转速和方向）
HAL_StatusTypeDef Emm42_SetVelocity(Chassis_Control_t *chassis, uint8_t motor_index, Rotation_Dir dir, int16_t velocity, uint8_t acceleration, uint8_t sync_flag)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return HAL_ERROR;  // 参数检查
    }

    // 设置控制模式为速度模式
    chassis->motors[motor_index].control_mode = CONTROL_MODE_VELOCITY;
    chassis->motors[motor_index].target_velocity = velocity;
    chassis->motors[motor_index].acceleration = acceleration;

    uint8_t data[8];                                 // 准备数据缓冲区
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_VELOCITY_CONTROL;                 // 功能码（速度控制）
    data[2] = dir;                                   // 方向
    data[3] = velocity & 0xFF;                       // 速度低字节
    data[4] = (velocity >> 8) & 0xFF;                // 速度高字节
    data[5] = acceleration;                          // 加速度
    data[6] = sync_flag;                             // 多机同步标志

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 7);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[8];
    memcpy(send_data, data, 7);  // 复制前7个字节
    send_data[7] = checksum;     // 添加校验字节

    // 发送消息并检查返回值
    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 8);

    if (status == HAL_OK) {
        chassis->motors[motor_index].is_moving = 1;
        chassis->motors[motor_index].position_reached = 0;
    } else {
        chassis->comm_ok = 0;
    }

    // 更新本地目标速度记录（考虑方向）
    chassis->motors[motor_index].target_velocity = (dir == DIR_CW) ? velocity : -velocity;

    return status;
}

// 设置电机位置
HAL_StatusTypeDef Emm42_SetPosition(Chassis_Control_t *chassis, uint8_t motor_index, Rotation_Dir dir, int16_t velocity, uint8_t acceleration, int32_t pulse_count, Position_Mode pos_mode, uint8_t sync_flag)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return HAL_ERROR;  // 参数检查
    }

    // 设置控制模式为位置模式
    chassis->motors[motor_index].control_mode = CONTROL_MODE_POSITION;
    chassis->motors[motor_index].target_position = pulse_count;
    chassis->motors[motor_index].pos_mode = pos_mode;

    uint8_t data[13];
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_POSITION_CONTROL;                 // 功能码（位置控制）
    data[2] = dir;                                   // 方向
    data[3] = velocity & 0xFF;                       // 速度低字节
    data[4] = (velocity >> 8) & 0xFF;                // 速度高字节
    data[5] = acceleration;                          // 加速度

    // 脉冲数（4字节，小端格式）
    data[6] = pulse_count & 0xFF;
    data[7] = (pulse_count >> 8) & 0xFF;
    data[8] = (pulse_count >> 16) & 0xFF;
    data[9] = (pulse_count >> 24) & 0xFF;

    data[10] = pos_mode;
    data[11] = sync_flag;

    // 计算校验和
    uint8_t checksum = Emm42_CalculateChecksum(chassis, data, 12);
    data[12] = checksum;

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, data, 13);

    if (status == HAL_OK) {
        chassis->motors[motor_index].is_moving = 1;
        chassis->motors[motor_index].position_reached = 0;
    } else {
        chassis->comm_ok = 0;
    }

    return status;
}

// 触发四个电机同步运动（使所有电机同时执行之前设置的命令）
HAL_StatusTypeDef Emm42_TriggerSync(Chassis_Control_t *chassis)
{
    if (chassis == NULL) {
        return HAL_ERROR;  // 参数检查
    }

    uint8_t data[4];               // 准备数据缓冲区
    data[0] = MOTOR_BROADCAST_ID;  // 默认使用地址1，0x00为广播地址，只有地址为1的电机会回复
    data[1] = FUNC_SYNC_MOTION;    // 功能码（同步运动）
    data[2] = 0x66;                // 固定值（协议要求）

    // 计算校验和
    uint8_t checksum = Emm42_CalculateChecksum(chassis, data, 3);
    data[3] = checksum;

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, 0x00, data, 4);

    if (status != HAL_OK) {
        chassis->comm_ok = 0;
    }

    return status;
}

// 立即停止电机
HAL_StatusTypeDef Emm42_StopMotor(Chassis_Control_t *chassis, uint8_t motor_index, uint8_t sync_flag)
{
    // 检查电机索引是否有效
    if (motor_index >= MOTOR_NUM) return;

    uint8_t data[5];                                 // 准备数据缓冲区
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_STOP_MOTOR;                       // 功能码（停止电机）
    data[2] = 0x98;                                  // 固定值（协议要求）
    data[3] = sync_flag;                             // 多机同步标志

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 4);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[5];
    memcpy(send_data, data, 4);  // 复制前4个字节
    send_data[4] = checksum;     // 添加校验字节

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 5);

    if (status == HAL_OK) {
        chassis->motors[motor_index].is_moving = 0;
        if (chassis->motors[motor_index].control_mode == CONTROL_MODE_VELOCITY) {
            chassis->motors[motor_index].target_velocity = 0;
        }
    } else {
        chassis->comm_ok = 0;
    }

    return status;
}

// 电机编码器转动角度清零（重置指定电机的编码器计数值）
HAL_StatusTypeDef Emm42_ClearEncoder(Chassis_Control_t *chassis, uint8_t motor_index)
{
    // 检查电机索引是否有效
    if (motor_index >= MOTOR_NUM) return;

    uint8_t data[4];                                 // 准备数据缓冲区
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_CLEAR_ENCODER;                    // 功能码（清零编码器）
    data[2] = 0x6D;                                  // 固定值（协议要求）

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 3);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[4];
    memcpy(send_data, data, 3);  // 复制前3个字节
    send_data[3] = checksum;     // 添加校验字节

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 4);

    if (status == HAL_OK) {
        chassis->motors[motor_index].actual_position = 0;
        chassis->motors[motor_index].target_position = 0;
    } else {
        chassis->comm_ok = 0;
    }

    return status;
}

// 读取电机实时转速
HAL_StatusTypeDef Emm42_ReadVelocity(Chassis_Control_t *chassis, uint8_t motor_index)
{
    // 检查电机索引是否有效
    if (motor_index >= MOTOR_NUM) return;

    uint8_t data[3];                                 // 准备数据缓冲区
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_READ_VELOCITY;                    // 功能码（读取转速）

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 2);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[3];
    memcpy(send_data, data, 2);  // 复制前2个字节
    send_data[2] = checksum;     // 添加校验字节

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 3);

    if (status != HAL_OK) {
        chassis->comm_ok = 0;
    }

    return status;
}

// 读取电机实时位置
HAL_StatusTypeDef Emm42_ReadPosition(Chassis_Control_t *chassis, uint8_t motor_index)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return HAL_ERROR;  // 参数检查
    }

    uint8_t data[3];                                 // 准备数据缓冲区
    data[0] = chassis->motors[motor_index].address;  // 地址
    data[1] = FUNC_READ_POSITION;                    // 功能码（读取位置）

    // 计算校验和并添加到数据末尾
    uint8_t checksum = Calculate_Checksum(chassis, data, 2);

    // 创建发送数据缓冲区（包含校验和）
    uint8_t send_data[3];
    memcpy(send_data, data, 2);  // 复制前2个字节
    send_data[2] = checksum;     // 添加校验字节

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 3);

    if (status != HAL_OK) {
        chassis->comm_ok = 0;
    }

    return status;
}

// 控制整个底盘运动
void Emm42_MoveChassis(Chassis_Control_t *chassis, int16_t vx, int16_t vy, int16_t vw)
{
    // 底盘运动学模型
    int16_t fl_speed = vx - vy + vw;  // 前左轮速度 = vx - vy + vw
    int16_t fr_speed = vx + vy - vw;  // 前右轮速度 = vx + vy - vw
    int16_t rl_speed = vx + vy + vw;  // 后左轮速度 = vx + vy + vw
    int16_t rr_speed = vx - vy - vw;  // 后右轮速度 = vx - vy - vw

    // 设置各电机速度（不触发同步）
    Emm42_SetVelocity(chassis, 0, (fl_speed >= 0) ? DIR_CW : DIR_CCW, abs(fl_speed), 10, 0);
    Emm42_SetVelocity(chassis, 1, (fr_speed >= 0) ? DIR_CW : DIR_CCW, abs(fr_speed), 10, 0);
    Emm42_SetVelocity(chassis, 2, (rl_speed >= 0) ? DIR_CW : DIR_CCW, abs(rl_speed), 10, 0);
    Emm42_SetVelocity(chassis, 3, (rr_speed >= 0) ? DIR_CW : DIR_CCW, abs(rr_speed), 10, 0);

    // 触发同步运动
    HAL_StatusTypeDef status =  Emm42_TriggerSync(chassis);

    if (status != HAL_OK) {
        chassis->comm_ok = 0;
    }

    return status;
}

// 检查电机位置是否到达（发送读取错误状态命令，电机会返回位置到达状态）
HAL_StatusTypeDef Emm42_CheckPositionReached(Chassis_Control_t *chassis, uint8_t motor_index)
{
    if (chassis == NULL || motor_index >= MOTOR_NUM) {
        return HAL_ERROR;  // 参数检查
    }

    uint8_t data[3];
    data[0] = chassis->motors[motor_index].address;
    data[1] = FUNC_READ_ERROR;

    // 计算校验和
    uint8_t checksum = Emm42_CalculateChecksum(chassis, data, 2);

    uint8_t send_data[3];
    memcpy(send_data, data, 2);
    send_data[2] = checksum;

    // 发送消息
    HAL_StatusTypeDef status = Emm42_SendCANMessage(chassis, chassis->motors[motor_index].address, send_data, 3);

    if (status != HAL_OK) {
        chassis->comm_ok = 0;
    }

    return status;
}

// 从CAN接收数据更新电机状态（处理从电机接收到的数据帧）
void Emm42_UpdateFromCAN(Chassis_Control_t *chassis, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (chassis == NULL || rx_header == NULL || data == NULL) {
        return;  // 参数检查
    }

    uint8_t address = data[0];    // 第一个字节是地址
    uint8_t func_code = data[1];  // 第二个字节是功能码

    // 查找电机索引
    uint8_t motor_index = 0xFF;  // 初始化为无效值
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (address == chassis->motors[i].address)
        {
            motor_index = i;  // 找到匹配的电机
            break;
        }
    }

    if (motor_index == 0xFF) return; // 不是目标电机ID，忽略此消息

    // 根据功能码解析数据
    switch (func_code)
    {
        case FUNC_READ_VELOCITY:
            // 解析实时转速（格式：地址+功能码+符号+速度+校验）
            if (rx_header->DataLength >= 5)  // 检查数据长度是否足够
            {
                // 符号位（0正，1负）
                uint8_t sign = data[2];
                // 组合速度值（低字节在前，高字节在后）
                int16_t velocity = (data[4] << 8) | data[3];
                // 根据符号位确定速度方向
                chassis->motors[motor_index].actual_velocity = (sign == 0x00) ? velocity : -velocity;
            }
            break;

        case FUNC_READ_POSITION:
            // 解析实时位置（格式：地址+功能码+符号+位置低字节+...+校验
            if (rx_header->DataLength >= 7) // 地址+功能码+符号+位置(4字节)+校验
            {
                // 符号位（0正，1负）
                uint8_t sign = data[2];
                // 组合位置值（4字节，低字节在前）
                int32_t position = (data[6] << 24) | (data[5] << 16) | (data[4] << 8) | data[3];
                // 根据符号位确定位置方向
                chassis->motors[motor_index].encoder_position = (sign == 0x00) ? position : -position;
            }
            break;

        case FUNC_VELOCITY_CONTROL:
            // 速度控制响应（格式：地址+功能码+状态+校验）
            if (rx_header->DataLength >= 4)  // 检查数据长度是否足够
            {
                uint8_t status = data[2];  // 状态码
                if (status != 0x02)        // 如果不是正确响应
                {
                    chassis->motors[motor_index].error_code = status;  // 记录错误代码
                }
            }
            break;

        default:
            break;
    }

    // 更新通信状态（收到任何有效消息都认为通信正常）
    chassis->comm_ok = 1;
}