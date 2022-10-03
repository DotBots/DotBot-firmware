#ifndef __SERVOS_H
#define __SERVOS_H

/**
 * @file servos.h
 * @addtogroup sailbot
 *
 * @brief  Module for controlling servos on Kyosho Fortune 612 SailBot.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

//=========================== public ======================================

void servos_init(void);
void servos_rudder_turn(int8_t angle);
void servos_sail_turn(int8_t angle);
void servos_set(int8_t rudder_angle, int8_t sail_angle);

#endif
