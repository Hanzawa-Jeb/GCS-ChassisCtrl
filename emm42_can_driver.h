#ifndef EMM42_CAN_DRIVER_H
#define EMM42_CAN_DRIVER_H

#include "stm32h7xx_hal.h"        // 包含的HAL库头文件
#include "stm32h7xx_hal_fdcan.h"  // 包含FDHAL库头文件
#include <stdint.h>               // 包含标准整数类型定义
#include <stdbool.h>              // 包含布尔类型定义

// 电机数量定义
#define MOTOR_NUM 4

// ID (根据实际配置修改)
#define MOTOR_BROADCAST_ID 0x00  // 广播地址
#define MOTOR_FL_ID        0x01  // 前左电机ID
#define MOTOR_FR_ID        0x02  // 前右电机ID
#define MOTOR_RL_ID        0x03  // 后左电机ID
#define MOTOR_RR_ID        0x04  // 后右电机ID
#define MOTOR_TOP_ID       0x05  // 顶部电机ID
#define MOTOR_STRETCH_ID   0x06  // 伸缩电机ID

// 功能码定义
#define FUNC_ENABLE_CONTROL   0xF3  // 电机使能控制
#define FUNC_VELOCITY_CONTROL 0xF6  // 速度模式控制
#define FUNC_POSITION_CONTROL 0xFD  // 位置模式控制
#define FUNC_STOP_MOTOR       0xFE  // 立即停止
#define FUNC_SYNC_MOTION      0xFF  // 多机同步运动
#define FUNC_CLEAR_ENCODER    0x0A  // 编码器清零
#define FUNC_READ_VELOCITY    0x35  // 读取实时转速
#define FUNC_READ_POSITION    0x36  // 读取实时位置
#define FUNC_READ_ERROR       0x3A  // 读取电机状态标志位

// 校验方式定义
typedef enum {
    CHECKSUM_FIXED = 0,    // 固定0x6B校验（默认方式）
    CHECKSUM_XOR,          // XOR校验（异或校验）
    CHECKSUM_CRC8          // CRC8校验（循环冗余校验）
} Checksum_Mode;

// 电机旋转方向定义
typedef enum {
    DIR_CW = 0x00,   // 顺时针
    DIR_CCW = 0x01   // 逆时针
} Rotation_Dir;

// 位置控制模式定义
typedef enum {
    POSITION_RELATIVE = 0x00,  // 相对位置模式
    POSITION_ABSOLUTE = 0x01   // 绝对位置模式
} Position_Mode;

// 电机控制模式定义
typedef enum {
    CONTROL_MODE_VELOCITY = 0,  // 速度控制模式
    CONTROL_MODE_POSITION       // 位置控制模式
} Control_Mode;

// 统一电机数据结构体
typedef struct {
    // 基本信息
    uint8_t address;            // 电机地址
    Control_Mode control_mode;  // 当前控制模式
    uint8_t is_enabled;         // 使能状态
    uint8_t error_code;         // 错误代码
    
    // 速度模式参数
    int16_t target_velocity;    // 目标速度(RPM)
    int16_t actual_velocity;    // 实际速度(RPM)
    uint8_t acceleration;       // 加速度档位
    
    // 位置模式参数
    int32_t target_position;    // 目标位置(脉冲数)
    int32_t actual_position;    // 实际位置(脉冲数)
    Position_Mode pos_mode;     // 位置模式(相对/绝对)
    
    // 状态标志
    uint8_t is_moving;          // 运动状态
    uint8_t position_reached;   // 位置到达标志
    uint8_t temperature;        // 电机温度
} Unified_Motor_Data_t;

// 底盘控制结构体
typedef struct {
    Motor_Data_t motors[MOTOR_NUM];  // 四个电机数据()
    Checksum_Mode checksum_mode;     // 校验方式
    uint8_t sync_flag;               // 同步触发标志
    uint8_t comm_ok;                 // 通信正常标志
    FDCAN_HandleTypeDef *hfdcan;     // FDCAN句柄
} Chassis_Control_t;

// 函数声明
void Emm42_Init(Chassis_Control_t *chassis, FDCAN_HandleTypeDef *hfdcan, Checksum_Mode checksum_mode);  // 初始化底盘电机驱动
void Emm42_EnableMotor(Chassis_Control_t *chassis, uint8_t motor_index, uint8_t enable);  // 电机使能控制
void Emm42_SetVelocity(Chassis_Control_t *chassis, uint8_t motor_index, Rotation_Dir dir, int16_t velocity, uint8_t acceleration, uint8_t sync_flag);  // 设置单个电机速度
void Emm42_TriggerSync(Chassis_Control_t *chassis);  // 触发四个电机同步运动
void Emm42_StopMotor(Chassis_Control_t *chassis, uint8_t motor_index, uint8_t sync_flag);  // 立即停止电机
void Emm42_ClearEncoder(Chassis_Control_t *chassis, uint8_t motor_index);  // 电机编码器转动角度清零
void Emm42_ReadVelocity(Chassis_Control_t *chassis, uint8_t motor_index);  // 读取电机实时转速
void Emm42_ReadPosition(Chassis_Control_t *chassis, uint8_t motor_index);  // 读取电机实时位置
void Emm42_MoveChassis(Chassis_Control_t *chassis, int16_t vx, int16_t vy, int16_t vw);  // 控制整个底盘运动
void Emm42_UpdateFromCAN(Chassis_Control_t *chassis, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);  // 从CAN接收数据更新电机状态

// 辅助函数
uint8_t Calculate_Checksum(Chassis_Control_t *chassis, uint8_t *data, uint8_t len);  // 计算校验和
void Emm42_SendCANMessage(Chassis_Control_t *chassis, uint32_t std_id, uint8_t *data, uint8_t len);  // 发送CAN消息

#endif /* EMM42_CAN_DRIVER_H */