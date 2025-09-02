#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

// Move the chassis linearly along y+ direction
// Input: distance in mm
// Output: computes the wheel rotation angles internally
int chassis_control_move_linear(int distance);

// Turn the chassis in place (CCW positive)
// Input: angle in degrees
// Output: computes the wheel rotation angles internally
int chassis_control_turn(int angle);

#endif // CHASSIS_CONTROL_H
