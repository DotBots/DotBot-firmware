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

//=========================== defines ==========================================

typedef struct {
    uint8_t timestamp[11];
    uint8_t valid;
    float   latitude;
    uint8_t latitude_N_S;
    float   longitude;
    uint8_t longitude_E_W;
    float   velocity;
    float   course;
    uint8_t datestamp[7];
    float   variation;
    uint8_t variation_E_W;
    uint8_t mode;
} nmea_gprmc_t;

//=========================== public ======================================

void          gps_init(void);
nmea_gprmc_t *gps_last_known_position(void);

#endif
