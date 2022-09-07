/**
 * @file servos.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for controlling servos on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include "pwm.h"
#include "servos.h"

//=========================== defines ===================================

#define RUDDER_POSITION_UPPER_BOUND 1840
#define RUDDER_POSITION_LOWER_BOUND 840

#define SAIL_POSITION_UPPER_BOUND 2100
#define SAIL_POSITION_LOWER_BOUND 1200

// Max value of the PWM counter register.
#define M_TOP 20000

// 2 PWM channels are used
#define PWM_CHANNELS (2)

enum servo_position {
    SERVO_RUDDER = 0,
    SERVO_SAILS  = 1,
};

//=========================== variables =========================================

static const gpio_t _pwm_pins[PWM_CHANNELS] = {
    { .port = 0, .pin = 29 },  // rudder servo
    { .port = 0, .pin = 28 },  // sail servo
};

/**
 *  @brief Initialization routine of the PWM module.
 */
void servos_init(void) {
    db_pwm_init(_pwm_pins, PWM_CHANNELS, M_TOP);
}

void servos_rudder_turn(int8_t angle) {
    uint16_t pwm_length;

    pwm_length = (angle * (RUDDER_POSITION_UPPER_BOUND - RUDDER_POSITION_LOWER_BOUND)) / 255;
    pwm_length += (RUDDER_POSITION_UPPER_BOUND + RUDDER_POSITION_LOWER_BOUND) / 2;

    // make sure imprecision in calculation does not cause a position out of bounds
    if (pwm_length > RUDDER_POSITION_UPPER_BOUND) {
        pwm_length = RUDDER_POSITION_UPPER_BOUND;
    }
    if (pwm_length < RUDDER_POSITION_LOWER_BOUND) {
        pwm_length = RUDDER_POSITION_LOWER_BOUND;
    }

    db_pwm_channel_set((uint8_t)SERVO_RUDDER, pwm_length);
}

void servos_sail_turn(int8_t angle) {
    uint16_t pwm_length;

    pwm_length = (angle * (SAIL_POSITION_UPPER_BOUND - SAIL_POSITION_LOWER_BOUND)) / 255;
    pwm_length += (SAIL_POSITION_UPPER_BOUND + SAIL_POSITION_LOWER_BOUND) / 2;

    // make sure imprecision in calculation does not cause a position out of bounds
    if (pwm_length > SAIL_POSITION_UPPER_BOUND) {
        pwm_length = SAIL_POSITION_UPPER_BOUND;
    }
    if (pwm_length < SAIL_POSITION_LOWER_BOUND) {
        pwm_length = SAIL_POSITION_LOWER_BOUND;
    }

    db_pwm_channel_set((uint8_t)SERVO_SAILS, pwm_length);
}
