#include <stdio.h>
#include <math.h>
#include "control.h"

#if defined(BOARD_DOTBOT_V3)
#define DB_MAX_PWM             (60)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.8)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (-1)   ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (0.6)
#elif defined(BOARD_DOTBOT_V2)
#define DB_MAX_PWM             (70)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.8)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (-1)   ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (0.6)
#else                                 // BOARD_DOTBOT_V1
#define DB_MAX_PWM             (70)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.9)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (20)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (1)    ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (0.6)
#endif

bool compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle) {
    float dx       = (float)next->x - (float)origin->x;
    float dy       = (float)next->y - (float)origin->y;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    *angle = (int16_t)(atan2f(dx, dy) * -1 * 180 / M_PI);  // atan2f returns angle in radians in [-PI, PI], we want it in [0, 360] with 0 being north and positive angles being clockwise
    return distance > DB_DIRECTION_THRESHOLD;
}

void update_control(robot_control_t *control) {
    float dx                 = (float)control->waypoint_x - (float)control->pos_x;
    float dy                 = (float)control->waypoint_y - (float)control->pos_y;
    float distance_to_target = sqrtf(powf(dx, 2) + powf(dy, 2));

    if ((uint32_t)(distance_to_target) < control->waypoint_threshold) {
        // Target waypoint is reached
        control->waypoint_idx++;
        return;
    }

    if (control->direction == DB_DIRECTION_INVALID) {
        // Unknown direction, just move forward a bit
        control->pwm_left  = (int16_t)DB_MAX_PWM;
        control->pwm_right = (int16_t)DB_MAX_PWM;
        return;
    }

    coordinate_t next            = { .x = control->waypoint_x, .y = control->waypoint_y };
    coordinate_t origin          = { .x = control->pos_x, .y = control->pos_y };
    int16_t      angle_to_target = 0;
    if (!compute_angle(&origin, &next, &angle_to_target)) {
        angle_to_target = 0;
    }

    if (control->direction >= 180) {
        control->direction -= 360;
    } else if (control->direction < -180) {
        control->direction += 360;
    }

    float   angular_speed = 0;
    int16_t error_angle   = 0;
    error_angle           = angle_to_target - control->direction;
    if (error_angle >= 180) {
        error_angle -= 360;
    } else if (error_angle < -180) {
        error_angle += 360;
    }

    float speedReductionFactor = 1.0;  // No reduction by default
    if ((uint32_t)(distance_to_target) < control->waypoint_threshold * 3) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }

    if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }
    angular_speed      = (float)(error_angle / 180.0f) * DB_MAX_PWM * DB_ANGULAR_SIDE_FACTOR * DB_ANGULAR_SPEED_GAIN;
    control->pwm_left  = (int16_t)(((DB_MAX_PWM * speedReductionFactor) - angular_speed));
    control->pwm_right = (int16_t)(((DB_MAX_PWM * speedReductionFactor) + angular_speed));

    // printf(
    //     "Loop data - direction: %i - "
    //     "dx: %f - dy: %f - distance to target: %f - "
    //     "angle to target: %i - error angle: %i - angular speed: %f - "
    //     "pwm_left: %i - pwm_right: %i\n",
    //     control->direction,
    //     distance_to_target, dx, dy,
    //     angle_to_target, error_angle, angular_speed,
    //     left_speed, right_speed
    // );
}
