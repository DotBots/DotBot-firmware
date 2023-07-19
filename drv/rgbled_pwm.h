#ifndef __RGBLED_PWM_H
#define __RGBLED_PWM_H

/**
 * @file rgbled_pwm.h
 * @addtogroup DRV
 *
 * @brief  Cross-platform declaration "rgbled pwm" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdlib.h>
#include <stdint.h>

#include "gpio.h"
#include "pwm.h"

//=========================== definitions ======================================

typedef struct {
    const gpio_t pins[3];  // red: 0, green: 1, blue: 2
} db_rgbled_pwm_conf_t;

//=========================== public ===========================================

/**
 * @brief   Initialize the RGB LED pins
 *
 * @param[in]   conf    Configuration for the RGB LED pins
 */
void db_rgbled_pwm_init(const db_rgbled_pwm_conf_t *conf);

/**
 * @brief   Set the color of the RGB LED
 *
 * @param[output]   payload     Decoded payload contained in the input buffer
 */
void db_rgbled_pwm_set_color(uint8_t red, uint8_t green, uint8_t blue);

#endif
