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
    float   longitude;
    float   velocity;
    float   course;
    uint8_t datestamp[7];
    float   variation;
    uint8_t mode;
} nmea_gprmc_t;

typedef void (*gps_rx_cb_t)(nmea_gprmc_t *data);  ///< Callback function prototype, it is called on each $GPRMC sentence received

//=========================== public ======================================

/**
 *  @brief Initialization routine of the GPS module.
 */
void gps_init(gps_rx_cb_t callback);

/**
 *  @brief Routine that returns the last known position of the GPS module.
 *  @return Last known position.
 */
nmea_gprmc_t *gps_last_known_position(void);

#endif
