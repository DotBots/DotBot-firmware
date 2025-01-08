#ifndef __RGBLED_H
#define __RGBLED_H

/**
 * @defgroup    drv_rgbled      RGB LED driver
 * @ingroup     drv
 * @brief       Control the rgbled driver (through SPI)
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <nrf.h>

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
 * @param[in] red   red value of the led color [0 - 255]
 * @param[in] green green value of the led color [0 - 255]
 * @param[in] blue  blue value of the led color [0 - 255]
 */
void db_rgbled_set(uint8_t red, uint8_t green, uint8_t blue);

#endif  // __RGBLED_H
