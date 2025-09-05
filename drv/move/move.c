/**
 * @file
 * @ingroup drv_move
 *
 * @brief  Implemenation of the "move" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "board.h"
#include "board_config.h"
#include "motors.h"
#include "qdec.h"
#include "timer.h"

#include "move.h"

//=========================== define ===========================================

#define REFRESH_DELAY_MS       (10)  //< ms
#define DOTBOT_MOTOR_REDUCTION (50)  //< Motor reduction factor
#define DOTBOT_ENCODER_CPR     (12)  //< Encoder count per rotation
#define DOTBOT_WHEEL_RADIUS    (20)  //< mm
#define DOTBOT_RADIUS          (45)  //< mm
#define QDEC_LEFT              (0)
#define QDEC_RIGHT             (1)
#define MOTOR_OFFSET           (5)
#define MOVE_TIMER_DEV         (0)

//=========================== variables ========================================

static const qdec_conf_t qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t qdec_right = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

static int32_t _previous_left_accumulator  = 0;
static int32_t _previous_right_accumulator = 0;

//=========================== private ==========================================

static float _distance_from_angle(uint16_t angle) {
    return ((float)angle * DOTBOT_RADIUS * M_PI) / 180;
}

static void _move_reset(void) {
    db_qdec_read_and_clear(QDEC_LEFT);
    db_qdec_read_and_clear(QDEC_RIGHT);
    _previous_left_accumulator  = 0;
    _previous_right_accumulator = 0;
}

//=========================== public ===========================================

void db_move_init(void) {
    db_board_init();
    db_motors_init();

    db_timer_init(MOVE_TIMER_DEV);

    db_qdec_init(QDEC_LEFT, &qdec_left, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &qdec_right, NULL, NULL);
}

void db_move_deinit(void) {
    db_timer_stop(MOVE_TIMER_DEV);
}

void db_move_straight(uint16_t distance, int8_t speed) {
    _move_reset();

    if (distance == 0 || speed == 0) {
        return;
    }

    int8_t   left_power        = speed;
    int8_t   right_power       = speed;
    uint32_t offset            = 5;
    int32_t  expected_distance = distance;
    if (speed < 0) {
        offset *= -1;
        expected_distance *= 1;
    }

    float correction = -1.0;
    if (speed > 0) {
        expected_distance += correction;
    } else if (speed < 0) {
        expected_distance -= correction;
    }

    int16_t left_count       = 0;
    int16_t right_count      = 0;
    int16_t prev_left_count  = 0;
    int16_t prev_right_count = 0;
    int16_t left_diff, right_diff;
    float   counts_per_rev      = DOTBOT_ENCODER_CPR * DOTBOT_MOTOR_REDUCTION;
    float   wheel_diameter      = DOTBOT_WHEEL_RADIUS * 2;
    float   wheel_circumference = M_PI * wheel_diameter;
    float   rev_count           = expected_distance / wheel_circumference;
    float   target_count        = rev_count * counts_per_rev;
    db_motors_set_speed(left_power, right_power);

    while (abs(right_count) < fabs(target_count)) {
        left_count       = db_qdec_read(QDEC_LEFT);
        right_count      = db_qdec_read(QDEC_RIGHT);
        left_diff        = abs(left_count - prev_left_count);
        right_diff       = abs(right_count - prev_right_count);
        prev_left_count  = left_count;
        prev_right_count = right_count;
        if (left_diff > right_diff) {
            left_power  = left_power - offset;
            right_power = right_power + offset;
        } else if (left_diff < right_diff) {
            left_power  = left_power + offset;
            right_power = right_power - offset;
        }
        db_motors_set_speed(left_power, right_power);
        db_timer_delay_ms(MOVE_TIMER_DEV, REFRESH_DELAY_MS);
    }

    db_motors_set_speed(0, 0);
}

void db_move_rotate(uint16_t angle, int8_t speed) {
    _move_reset();

    if (angle == 0 || speed == 0) {
        return;
    }

    int8_t   left_power  = speed;
    int8_t   right_power = speed * -1;
    uint32_t offset      = 5;
    // Compute distance from angle
    int32_t expected_distance = _distance_from_angle(angle);
    if (speed < 0) {
        offset *= -1;
        expected_distance *= 1;
    }

    float correction = -5.0;
    if (speed > 0) {
        expected_distance += correction;
    } else if (speed < 0) {
        expected_distance -= correction;
    }

    int16_t left_count       = 0;
    int16_t right_count      = 0;
    int16_t prev_left_count  = 0;
    int16_t prev_right_count = 0;
    int16_t left_diff, right_diff;
    float   counts_per_rev      = DOTBOT_ENCODER_CPR * DOTBOT_MOTOR_REDUCTION;
    float   wheel_diameter      = DOTBOT_WHEEL_RADIUS * 2;
    float   wheel_circumference = M_PI * wheel_diameter;
    float   rev_count           = expected_distance / wheel_circumference;
    float   target_count        = rev_count * counts_per_rev;
    db_motors_set_speed(left_power, right_power);

    while (abs(right_count) < fabs(target_count)) {
        left_count       = db_qdec_read(QDEC_LEFT);
        right_count      = db_qdec_read(QDEC_RIGHT);
        left_diff        = abs(left_count - prev_left_count);
        right_diff       = abs(right_count - prev_right_count);
        prev_left_count  = left_count;
        prev_right_count = right_count;
        if (left_diff > right_diff) {
            left_power -= offset;
            right_power -= offset;
        } else if (left_diff < right_diff) {
            left_power += offset;
            right_power += offset;
        }
        db_motors_set_speed(left_power, right_power);
        db_timer_delay_ms(MOVE_TIMER_DEV, REFRESH_DELAY_MS);
    }

    db_motors_set_speed(0, 0);
}
