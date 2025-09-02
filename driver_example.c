// 主程序中示例用法
Chassis_Control_t chassis;

int main(void)
{
    // HAL初始化等代码...
    
    // 初始化底盘电机驱动（使用XOR校验）
    Emm42_Init(&chassis, &hcan1, CHECKSUM_XOR);
    
    // 使能所有电机
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        Emm42_EnableMotor(&chassis, i, 1);
        HAL_Delay(10);
    }
    
    while (1)
    {
        // 控制底盘以速度100向前移动
        Emm42_MoveChassis(&chassis, 100, 0, 0);
        
        // 读取所有电机速度
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            Emm42_ReadVelocity(&chassis, i);
        }
        
        HAL_Delay(100);
    }
}

// FDCAN接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t data[8];
        
        // 从FDCAN接收消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK)
        {
            // 处理接收到的CAN消息
            Emm42_UpdateFromCAN(&chassis, &rx_header, data);
        }
    }
}