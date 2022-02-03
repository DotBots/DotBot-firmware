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
 * @copyright INRIA, 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>


//=========================== prototypes ======================================

//=========================== public ======================================
void db_motors_init(void);
void db_motors_setSpeed(int16_t l_speed, int16_t r_speed);

#endif