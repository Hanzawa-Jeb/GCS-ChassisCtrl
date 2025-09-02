#include "chassis_control_top.h"
#include "chassis_control.h"
#include <stdlib.h>

int controlTop(int x, int y, int angle)
{
    //in this function, we separate the whole process into four stages
    //y_move->turn->x_move->final_turn
    int curr_angle = 0;          //current angle relative to initial position

    //Stage 1: Move along y axis
    if(y > 0) {
        chassis_control_turn(0 - curr_angle);     //turn to face forward
        curr_angle = 0;
    } else {
        chassis_control_turn(180 - curr_angle);   //turn to face backward
        curr_angle = 180;
    }
    chassis_control_move_linear(abs(y));

    //Stage 2: Turn for x movement
    if(x > 0) {
        chassis_control_turn(90 - curr_angle);    //turn to face left
        curr_angle = 90;
    } else {
        chassis_control_turn(-90 - curr_angle);   //turn to face right
        curr_angle = -90;
    }

    //Stage 3: Move along x axis
    chassis_control_move_linear(abs(x));

    //Stage 4: Final turn to target angle relative to initial position
    chassis_control_turn(angle - curr_angle);     //compensate for previous rotations

    return 0;
}