#ifndef __PWM_H
#define __PWM_H

/**
 * @defgroup    bsp_pwm PWM
 * @ingroup     bsp
 * @brief       Control the PWM peripherals
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <stdlib.h>

#include <nrf.h>

#include "gpio.h"

//=========================== defines ==========================================

typedef uint8_t pwm_t;  ///< PWM peripheral index

//=========================== public ===========================================

/**
 * @brief Initialize the PWM interface (uses the PWM0 peripheral)
 *
 * PWM frequency = 10Khz
 * PWM resolution = 100 units (1us resolution)
 *
 * The number of channels must not exceed the size of the array containing
 * the pins.
 *
 * @param[in] pwm           index of the PWM peripheral to initialize
 * @param[in] pins          pointer to array of pointer to GPIO pins
 * @param[in] num_channels  number of channels to configure
 * @param[in] mtop          max value of the PWM counter register
 */
void db_pwm_init(pwm_t pwm, const gpio_t *pins, size_t num_channels, uint16_t mtop);

/**
 * @brief Set the value of a PWM channel
 *
 * The channel value must be lower or equal to 3.
 *
 * @param[in] pwm       index of the PWM peripheral
 * @param[in] channel   index of channel to set
 * @param[in] value     value to set to the channel
 */
void db_pwm_channel_set(pwm_t pwm, uint8_t channel, uint16_t value);

/**
 * @brief Set the values of all PWM channels
 *
 * The array of values must be 4 bytes long exactly.
 *
 * @param[in] pwm       index of the PWM peripheral
 * @param[in] values    pointer to the array containing the values to set
 */
void db_pwm_channels_set(pwm_t pwm, uint16_t *values);

#endif
