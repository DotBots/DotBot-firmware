#ifndef __RGBLED_PWM_H
#define __RGBLED_PWM_H

/**
 * @defgroup    drv_rgbled_pwm  RGB LED driver via PWM
 * @ingroup     drv
 * @brief       Driver for RGB LED via PWM
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>

#include "gpio.h"
#include "pwm.h"

//=========================== definitions ======================================

/// RGB LED PWM configuration
typedef struct {
    pwm_t        pwm;      // Index of PWM to use
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
 * @param[in]   red     Red value [0 - 255]
 * @param[in]   green   Green value [0 - 255]
 * @param[in]   blue    Blue value [0 - 255]
 */
void db_rgbled_pwm_set_color(uint8_t red, uint8_t green, uint8_t blue);

#endif
