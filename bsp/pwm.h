#ifndef __PWM_H
#define __PWM_H

/**
 * @file pwm.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "pwm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"

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
 * @param[in] pins          pointer to array of pointer to GPIO pins
 * @param[in] num_channels  number of channels to configure
 */
void db_pwm_init(const gpio_t *pins, size_t num_channels);

/**
 * @brief Set the value of a PWM channel
 *
 * The channel value must be lower or equal to 3.
 *
 * @param[in] channel   index of channel to set
 * @param[in] value     value to set to the channel
 */
void db_pwm_channel_set(uint8_t channel, uint16_t value);

/**
 * @brief Set the values of all PWM channels
 *
 * The array of values must be 4 bytes long exactly.
 *
 * @param[in] values    pointer to the array containing the values to set
 */
void db_pwm_channels_set(uint16_t *values);

#endif
