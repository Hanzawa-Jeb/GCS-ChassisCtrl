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

// 下发四轮位置（角度-度），相对位置模式，统一触发
// 需在上层已完成 Emm42_Init/EnableMotor
struct Chassis_Control_t; // forward decl to avoid header include cycle
void chassis_control_send_angles(struct Chassis_Control_t *chassis,
                                 double angle_1, double angle_2, double angle_3, double angle_4,
                                 int16_t velocity, uint8_t acceleration);

#endif // CHASSIS_CONTROL_H
