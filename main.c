#include "main.h"

#include <stdio.h>
#include <stdlib.h>

//我们在这里假设已经包含了必要的HAL库头文件
//我们假设有若干个运动存储在一个序列当中，定义一个struct来存储这些序列对应的x, y, 角度要求

int main() {

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



    return 0;
}