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

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== prototypes ======================================

void db_rgbled_init(void);
void db_rgbled_set(uint8_t r, uint8_t g, uint8_t b);


#endif