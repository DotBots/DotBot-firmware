#ifndef __MOTORS_H
#define __MOTORS_H

/**
 * @file motors.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "motors" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

//=========================== prototypes ======================================

//=========================== public ======================================

void db_motors_init(void);
void db_motors_set_speed(int16_t l_speed, int16_t r_speed);

#endif
