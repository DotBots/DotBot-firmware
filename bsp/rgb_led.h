#ifndef __RGB_LEDS_H
#define __RGB_LEDS_H

/**
 * @file rgb_led.h
 * @addtogroup BSP
 * 
 * @brief  Cross-platform declaration "leds" bsp module.
 * 
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * 
 * @copyright INRIA, 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>


//=========================== prototypes ======================================
void init_rgb_led(void);
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b);


#endif