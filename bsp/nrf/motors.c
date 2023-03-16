/**
 * @file motors.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "motors bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "gpio.h"
#include "motors.h"
#include "pwm.h"

//=========================== defines ==========================================

// Max value of the PWM counter register (100us => 10kHz)
#define M_TOP (100)
// 4 PWM channels are used
#define PWM_CHANNELS (4)

//=========================== variables =========================================

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
static const gpio_t _pwm_pins[PWM_CHANNELS] = {
    { .port = 0, .pin = 28 },  // AIN1
    { .port = 0, .pin = 29 },  // AIN2
    { .port = 0, .pin = 30 },  // BIN1
    { .port = 0, .pin = 31 },  // BIN2
};
#else
static const gpio_t _pwm_pins[PWM_CHANNELS] = {
    { .port = 0, .pin = 2 },   // AIN1
    { .port = 0, .pin = 28 },  // AIN2
    { .port = 1, .pin = 9 },   // BIN1
    { .port = 0, .pin = 11 },  // BIN2
};
#endif

//=========================== public ==========================================

void db_motors_init(void) {
    db_pwm_init(_pwm_pins, PWM_CHANNELS, M_TOP);
}

void db_motors_set_speed(int16_t l_speed, int16_t r_speed) {

    // Double check for out-of-bound values.
    if (l_speed > 100)
        l_speed = 100;
    if (r_speed > 100)
        r_speed = 100;

    if (l_speed < -100)
        l_speed = -100;
    if (r_speed < -100)
        r_speed = -100;

    uint16_t pwm_seq[PWM_CHANNELS] = { 0 };

    // Left motor processing
    if (l_speed >= 0)  // Positive values turn the motor forward.
    {
        pwm_seq[0] = l_speed;
        pwm_seq[1] = 0;
    }
    if (l_speed < 0)  // Negative values turn the motor backward.
    {
        l_speed *= -1;  // remove the negative before loading into memory

        pwm_seq[0] = 0;
        pwm_seq[1] = l_speed;
    }

    // Right motor processing
    if (r_speed >= 0)  // Positive values turn the motor forward.
    {
        pwm_seq[2] = r_speed;
        pwm_seq[3] = 0;
    }
    if (r_speed < 0)  // Negative values turn the motor backward.
    {
        r_speed *= -1;  // remove the negative before loading into memory

        pwm_seq[2] = 0;
        pwm_seq[3] = r_speed;
    }

    // Update PWM values
    db_pwm_channels_set(pwm_seq);
}
