#include "main.h"
#include "chassis_control_top.h"
#include "emm42_can_driver.h"

#include <stdio.h>
#include <stdlib.h>

//我们假设有若干个运动存储在一个序列当中，定义一个struct来存储这些序列对应的x, y, 角度要求

typedef struct action {
    int x;
    //the relative x coordinate
    int y;
    //the relative y coordinate
    int angle;
    //the relative angle
} OneMove;
//the parameter of a single move

// FDCAN 句柄由底层生成（CubeMX等）。这里声明为外部符号供初始化使用。
extern FDCAN_HandleTypeDef hfdcan1;

// 全局底盘控制句柄
Chassis_Control_t chassis;

// 封装的底盘初始化函数
static void chassis_init(void)
{
	// 若在真实板卡环境，可在此调用以下初始化（由HAL/Cube生成）
	// HAL_Init();
	// SystemClock_Config();
	// MX_FDCAN1_Init();

	// 1) 驱动初始化（选择校验模式）
	Emm42_Init(&chassis, &hfdcan1, CHECKSUM_XOR);

	// 2) 依次使能四个电机
	for (uint8_t i = 0; i < MOTOR_NUM; i++)
	{
		Emm42_EnableMotor(&chassis, i, 1);
		HAL_Delay(10);
	}

	// 3) 可选：清零编码器，便于相对位置控制统一零点
	for (uint8_t i = 0; i < MOTOR_NUM; i++)
	{
		Emm42_ClearEncoder(&chassis, i);
		HAL_Delay(5);
	}
}

int main() {

	// 初始化底盘（CAN驱动+电机）
	chassis_init();

    int MoveCnt = 0;
    //the count of the total move

    int curr_x = 0;
    int curr_y = 0;
    int curr_angle = 0;
    //initialize the initial position

    OneMove* MoveSeq = (OneMove *)calloc(MoveCnt, sizeof(OneMove));
    //this is the sequence of the moves
    //for every move, we need to be sure what to be done at each step
    //for example, we only show what to be done for one step

    // 示例：若实际有动作序列，请填充 MoveSeq 并调用 controlTop
    // controlTop(MoveSeq[0].x, MoveSeq[0].y, MoveSeq[0].angle);

    //only for demonstration, in practical, we need to fill the MoveSeq.

    return 0;
}