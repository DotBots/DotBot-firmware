/**
 * @file move.c
 * @addtogroup DRV
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

//=========================== variables ========================================

static const qdec_conf_t qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

//=========================== private ==========================================

static float _distance_from_accumulator(int16_t accumulator) {
    return (((float)accumulator / DOTBOT_ENCODER_CPR) * 2 * M_PI * DOTBOT_WHEEL_RADIUS) / DOTBOT_MOTOR_REDUCTION;
}

static float _angle_from_accumulator(int16_t accumulator) {
    return (180 * _distance_from_accumulator(accumulator)) / (M_PI * DOTBOT_RADIUS);
}

static void _move_reset(void) {
    db_qdec_read_and_clear(QDEC_LEFT);
}

//=========================== public ===========================================

void db_move_init(void) {
    db_board_init();
    db_motors_init();

    db_timer_init();

    db_qdec_init(QDEC_LEFT, &qdec_left, NULL, NULL);
}

void db_move_straight(uint16_t distance, int8_t speed) {
    _move_reset();

    if (distance == 0) {
        return;
    }

    int32_t accumulator = 0;
    db_motors_set_speed(speed, speed);
    while (1) {
        accumulator += db_qdec_read_and_clear(QDEC_LEFT);
        float current_distance = fabs(_distance_from_accumulator(accumulator));
        if ((current_distance > (float)distance)) {
            break;
        }
        db_timer_delay_ms(REFRESH_DELAY_MS);
    }

    db_motors_set_speed(0, 0);
}

void db_move_rotate(uint16_t angle, int8_t speed) {
    _move_reset();
    if (angle == 0) {
        return;
    }

    int32_t accumulator = 0;
    db_motors_set_speed(speed, speed * -1);
    while (1) {
        accumulator += db_qdec_read_and_clear(QDEC_LEFT);
        float current_angle = fabs(_angle_from_accumulator(accumulator));
        if ((current_angle > (float)angle)) {
            break;
        }

        db_timer_delay_ms(REFRESH_DELAY_MS);
    }

    db_motors_set_speed(0, 0);
}
