#include "chassis_control.h"
#include "emm42_can_driver.h"
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

    // Send to motors (relative + sync)
    extern Chassis_Control_t chassis;
    chassis_control_send_angles(&chassis,
                                (double)angle_1, (double)angle_2,
                                (double)angle_3, (double)angle_4,
                                200, 10);

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

    // Send to motors (relative + sync)
    extern Chassis_Control_t chassis;
    chassis_control_send_angles(&chassis,
                                (double)angle_1, (double)angle_2,
                                (double)angle_3, (double)angle_4,
                                200, 10);

    return 0; // success
}

static int32_t degrees_to_pulses(double degrees)
{
    const double pulses_per_rev = 3200.0;
    double pulses = fabs(degrees) / 360.0 * pulses_per_rev;
    return (int32_t)llround(pulses);
}

static Rotation_Dir dir_from_degrees(double degrees)
{
    return (degrees >= 0.0) ? DIR_CW : DIR_CCW;
}

void chassis_control_send_angles(Chassis_Control_t *chassis,
                                 double angle_1, double angle_2, double angle_3, double angle_4,
                                 int16_t velocity, uint8_t acceleration)
{
    if (chassis == NULL) {
        return;
    }

    Emm42_SetPosition(chassis, 0, dir_from_degrees(angle_1), velocity, acceleration,
                      degrees_to_pulses(angle_1), POSITION_RELATIVE, 1);
    Emm42_SetPosition(chassis, 1, dir_from_degrees(angle_2), velocity, acceleration,
                      degrees_to_pulses(angle_2), POSITION_RELATIVE, 1);
    Emm42_SetPosition(chassis, 2, dir_from_degrees(angle_3), velocity, acceleration,
                      degrees_to_pulses(angle_3), POSITION_RELATIVE, 1);
    Emm42_SetPosition(chassis, 3, dir_from_degrees(angle_4), velocity, acceleration,
                      degrees_to_pulses(angle_4), POSITION_RELATIVE, 1);

    Emm42_TriggerSync(chassis);
}
