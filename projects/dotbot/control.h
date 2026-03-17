#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <math.h>

/// Coordinate struct
typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} coordinate_t;

/// Robot control struct
typedef struct {
    uint32_t pos_x;               ///< X coordinate of the robot in mm
    uint32_t pos_y;               ///< Y coordinate of the robot in mm
    int16_t  direction;           ///< Direction of the robot in radians, in [0, 2 * PI]
    uint8_t  waypoints_length;    ///< Number of waypoints in the waypoints array
    uint8_t  waypoint_idx;        ///< Index of the current target waypoint in the waypoints array
    uint32_t waypoint_x;          ///< X coordinate of the current target waypoint in mm
    uint32_t waypoint_y;          ///< Y coordinate of the current target waypoint in mm
    uint32_t waypoint_threshold;  ///< Distance threshold to consider a waypoint reached, in mm
    int8_t   pwm_left;            ///< PWM value for the left motor, in [-DB_MAX_SPEED, DB_MAX_SPEED]
    int8_t   pwm_right;           ///< PWM value for the right motor, in [-DB_MAX_SPEED, DB_MAX_SPEED]
} robot_control_t;

/**
 * @brief Compute the angle between the robot's current position and a target position
 *
 * @param origin    Pointer to the robot's current coordinate
 * @param next      Pointer to the target coordinate
 * @param angle     Pointer to the variable where the computed angle will be stored, in degrees, in [0, 360], with 0 being north and positive angles being clockwise
 */
void compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle);

/**
 * @brief Update the robot's control variables based on the current position, direction, and target waypoint
 *
 * @param control   Pointer to the robot_control_t struct containing the current control variables. The pwm_left and pwm_right fields will be updated by this function.
 */
void update_control(robot_control_t *control);

#endif  // __CONTROL_H
