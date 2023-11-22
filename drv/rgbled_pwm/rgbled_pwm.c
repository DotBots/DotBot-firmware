/**
 * @file
 * @ingroup drv_rgbled_pwm
 *
 * @brief  Implementation of the "rgbled pwm" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "rgbled_pwm.h"
#include "pwm.h"

//=========================== definitions ======================================

#define PWM_CHANNELS  (3)
#define PWM_MAX_VALUE (UINT8_MAX)

//=========================== public ===========================================

void db_rgbled_pwm_init(const db_rgbled_pwm_conf_t *conf) {
    db_pwm_init(conf->pwm, conf->pins, PWM_CHANNELS, PWM_MAX_VALUE);
    uint16_t pwm_seq[PWM_CHANNELS] = { 0, 0, 0 };
    db_pwm_channels_set(1, pwm_seq);
}

void db_rgbled_pwm_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t pwm_seq[PWM_CHANNELS] = { UINT8_MAX - red, UINT8_MAX - green, UINT8_MAX - blue };
    db_pwm_channels_set(1, pwm_seq);
}
