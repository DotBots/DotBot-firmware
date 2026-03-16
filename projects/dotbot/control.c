#include <stdio.h>
#include "control.h"

#define DB_DIRECTION_THRESHOLD (50)     ///< Threshold to update the direction (50mm)
#define DB_DIRECTION_INVALID   (-1000)  ///< Invalid angle e.g out of [0, 360] range
#if defined(BOARD_DOTBOT_V3)
#define DB_MAX_SPEED            (60)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR  (0.7)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (25)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (-1)   ///< Angular side factor
#elif defined(BOARD_DOTBOT_V2)
#define DB_MAX_SPEED            (70)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR  (0.8)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (35)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (-1)   ///< Angular side factor
#else                                  // BOARD_DOTBOT_V1
#define DB_MAX_SPEED            (70)   ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR  (0.9)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (20)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (30)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (1)    ///< Angular side factor
#endif

void compute_angle(const coordinate_t *next, const coordinate_t *origin, float *angle) {
    float dx       = (float)origin->x - (float)next->x;
    float dy       = (float)origin->y - (float)next->y;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    if (distance < DB_DIRECTION_THRESHOLD) {
        return;
    }

    *angle = atan2f(dy, dx);
}

void update_control(robot_control_t *control) {
    if (control->waypoint_idx >= control->waypoints_length) {
        printf("Last waypoint reached, exiting\n");
        control->pwm_left  = 0;
        control->pwm_right = 0;
        return;
    }

    if (control->direction >= M_PI) {
        control->direction -= 2 * M_PI;
    } else if (control->direction < -M_PI) {
        control->direction += 2 * M_PI;
    }

    float        dx                 = (float)control->pos_x - (float)control->waypoint_x;
    float        dy                 = (float)control->pos_y - (float)control->waypoint_y;
    float        distance_to_target = sqrtf(powf(dx, 2) + powf(dy, 2));
    coordinate_t next               = { .x = control->waypoint_x, .y = control->waypoint_y };
    coordinate_t origin             = { .x = control->pos_x, .y = control->pos_y };
    float        angle_to_target    = 0;
    compute_angle(&origin, &next, &angle_to_target);

    float speedReductionFactor = 1.0;  // No reduction by default
    if ((uint32_t)(distance_to_target) < control->waypoint_threshold * 2) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }

    int16_t left_speed    = 0;
    int16_t right_speed   = 0;
    float   angular_speed = 0;
    float   error_angle   = 0;
    if ((uint32_t)(distance_to_target) < control->waypoint_threshold) {
        // Target waypoint is reached
        control->waypoint_idx++;
    } else if (control->direction == DB_DIRECTION_INVALID) {
        // Unknown direction, just move forward a bit
        left_speed  = (int16_t)DB_MAX_SPEED * speedReductionFactor;
        right_speed = (int16_t)DB_MAX_SPEED * speedReductionFactor;
    } else {
        error_angle = fmodf((angle_to_target - control->direction + M_PI), 2 * M_PI);
        if (error_angle > M_PI) {
            error_angle -= 2 * M_PI;
        } else if (error_angle < -M_PI) {
            error_angle += 2 * M_PI;
        }

        int16_t error_angle_degrees = error_angle * 180 / M_PI;
        if (error_angle_degrees > DB_REDUCE_SPEED_ANGLE || error_angle_degrees < -DB_REDUCE_SPEED_ANGLE) {
            speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
        }
        angular_speed = (int16_t)(error_angle * DB_ANGULAR_SPEED_FACTOR);
        left_speed    = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) + angular_speed * DB_ANGULAR_SIDE_FACTOR));
        right_speed   = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) - angular_speed * DB_ANGULAR_SIDE_FACTOR));
        if (left_speed > DB_MAX_SPEED) {
            left_speed = DB_MAX_SPEED;
        }
        if (right_speed > DB_MAX_SPEED) {
            right_speed = DB_MAX_SPEED;
        }
    }

    // printf(
    //     "Loop data - direction: %f (%i°) - "
    //     "dx: %f - dy: %f - distance to target: %f - "
    //     "angle to target: %f (%i) - error angle: %f (%i) - angular speed: %f - "
    //     "pwm_left: %i - pwm_right: %i\n",
    //     control->direction, (int16_t)(control->direction * 180 / M_PI),
    //     distance_to_target, dx, dy,
    //     angle_to_target, (int16_t)(angle_to_target * 180 / M_PI), error_angle, (int16_t)(error_angle * 180 / M_PI), angular_speed,
    //     left_speed, right_speed
    // );

    control->pwm_left  = left_speed;
    control->pwm_right = right_speed;
}
