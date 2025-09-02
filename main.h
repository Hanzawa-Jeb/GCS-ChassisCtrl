#ifndef MAIN_H
#define MAIN_H

#include <chassis_control_top.h>
//more to include ...

typedef struct action {
    int x;
    //the relative x coordinate
    int y;
    //the relative y coordinate
    int angle;
    //the relative angle
} OneMove;
//the parameter of a single move

#endif