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
#include <string.h>
#include "pwm.h"
#include "servos.h"

//=========================== defines ===================================

#define RUDDER_POSITION_UPPER_BOUND 1840
#define RUDDER_POSITION_LOWER_BOUND 840

#define SAIL_POSITION_UPPER_BOUND 2100
#define SAIL_POSITION_LOWER_BOUND 1400

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

//=========================== prototypes =========================================
uint16_t calculate_pwm_length(int8_t angle, uint16_t upper_bound, uint16_t lower_bound);

/**
 *  @brief Initialization routine of the PWM module.
 */
void servos_init(void) {
    db_pwm_init(0, _pwm_pins, PWM_CHANNELS, M_TOP);
}

void servos_set(int8_t rudder_angle, int8_t sail_angle) {
    uint16_t pwm[4];

    memset(pwm, 0, sizeof(pwm));
    pwm[SERVO_RUDDER] = calculate_pwm_length(rudder_angle, RUDDER_POSITION_UPPER_BOUND, RUDDER_POSITION_LOWER_BOUND);
    pwm[SERVO_SAILS]  = calculate_pwm_length(sail_angle, SAIL_POSITION_UPPER_BOUND, SAIL_POSITION_LOWER_BOUND);

    db_pwm_channels_set(0, pwm);
}

void servos_rudder_turn(int8_t angle) {
    db_pwm_channel_set(0, (uint8_t)SERVO_RUDDER, calculate_pwm_length(angle, RUDDER_POSITION_UPPER_BOUND, RUDDER_POSITION_LOWER_BOUND));
}

void servos_sail_turn(int8_t angle) {
    db_pwm_channel_set(0, (uint8_t)SERVO_SAILS, calculate_pwm_length(angle, SAIL_POSITION_UPPER_BOUND, SAIL_POSITION_LOWER_BOUND));
}

//=========================== private =========================================

uint16_t calculate_pwm_length(int8_t angle, uint16_t upper_bound, uint16_t lower_bound) {
    uint16_t pwm_length;

    pwm_length = (angle * (upper_bound - lower_bound)) / 255;
    pwm_length += (upper_bound + lower_bound) / 2;

    // make sure imprecision in calculation does not cause a position out of bounds
    if (pwm_length > upper_bound) {
        pwm_length = upper_bound;
    }
    if (pwm_length < lower_bound) {
        pwm_length = lower_bound;
    }

    return pwm_length;
}
