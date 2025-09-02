#include "chassis_control.h"
#include <math.h>

#define WHEEL_RADIUS 50.0       // mm (radius = diameter/2)
#define HALF_WHEELBASE 87.5     // mm (axis length / 2)
#define HALF_TRACK 87.5         // mm (wheel distance / 2)
#define PI 3.14159265358979323846

int chassis_control_move_linear(int distance)
{
    int angle_1, angle_2, angle_3, angle_4;
    
    // Calculate wheel rotation angle for linear motion along y+ direction
    // X-type mecanum wheels: all wheels contribute equally along y+
    // Linear wheel rotation (radians) = distance / wheel radius
    double wheel_rotation_rad = distance / WHEEL_RADIUS; 
    
    // Convert to degrees
    double wheel_rotation_deg = wheel_rotation_rad * 180.0 / PI;

    // Assign to each wheel
    angle_1 = (int)wheel_rotation_deg; // Left Front
    angle_2 = (int)wheel_rotation_deg; // Right Front
    angle_3 = (int)wheel_rotation_deg; // Left Back
    angle_4 = (int)wheel_rotation_deg; // Right Back

    return 0; // success
}

int chassis_control_turn(int angle)
{
    int angle_1, angle_2, angle_3, angle_4;
    
    // Calculate wheel rotation angle for in-place rotation (CCW positive)
    // Rotation distance per wheel = (half_wheelbase + half_track) * pi * angle / 180
    double rotation_distance = (HALF_WHEELBASE + HALF_TRACK) * 2.0 * PI * angle / 360.0;

    // Wheel rotation in radians = rotation_distance / wheel_radius
    double wheel_rotation_rad = rotation_distance / WHEEL_RADIUS;

    // Convert to degrees
    double wheel_rotation_deg = wheel_rotation_rad * 180.0 / PI;

    // Assign wheel directions for X-type mecanum wheels
    // LF, RB: forward, RF, LB: backward for CCW rotation
    angle_1 = (int)wheel_rotation_deg;   // Left Front
    angle_2 = -(int)wheel_rotation_deg;  // Right Front
    angle_3 = -(int)wheel_rotation_deg;  // Left Back
    angle_4 = (int)wheel_rotation_deg;   // Right Back

    return 0; // success
}
