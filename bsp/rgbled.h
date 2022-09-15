#ifndef __RGBLEDS_H
#define __RGBLEDS_H

/**
 * @file rgbled.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "leds" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Configures the SPIM peripheral to work with the onboard DotBot RGB LED driver
 *
 * The DotBot uses a TLC5973D 1-wire RGB LED driver. the SPIM peripheral is used to generate
 * an arbitrary serial sequence that the Driver will recognize.
 *
 */
void db_rgbled_init(void);

/**
 * @brief Set the color to display in the RGB LED
 *
 *  Assembles the 1-wire encoded command to change the color of the RGB led.
 *  Afterward, an SPI transfer is necessary to actually update the LED color.
 *
 * @param[in] r red value of the led color [0 - 255]
 * @param[in] g green value of the led color [0 - 255]
 * @param[in] b blue value of the led color [0 - 255]
 */
void db_rgbled_set(uint8_t r, uint8_t g, uint8_t b);

#endif
