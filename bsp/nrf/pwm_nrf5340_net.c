/**
 * @file
 * @ingroup bsp_pwm
 *
 * @brief  nRF5340-network-specific definition of the "pwm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>

#include "gpio.h"

//=========================== public ===========================================

void db_pwm_init(const gpio_t *pins, size_t num_channels, uint16_t mtop) {
    (void)pins;
    (void)num_channels;
    (void)mtop;
}

void db_pwm_channel_set(uint8_t channel, uint16_t value) {
    (void)channel;
    (void)value;
}

void db_pwm_channels_set(uint16_t *values) {
    (void)values;
}
