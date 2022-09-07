#ifndef __GPS_H
#define __GPS_H

/**
 * @file gps.h
 * @addtogroup sailbot
 *
 * @brief  Module for reading the GPS on Kyosho Fortune 612 SailBot.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

//=========================== public ======================================

void gps_init(void);
void gps_last_known_position(void);

#endif
