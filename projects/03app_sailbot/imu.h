#ifndef __IMU_H
#define __IMU_H

/**
 * @file imu.h
 * @addtogroup sailbot
 *
 * @brief  Module for controlling the IMU on the SailBot.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <nrf.h>

typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lis3mdl_compass_data_t;

void  imu_init(void);
bool  imu_data_ready(void);
float imu_read_heading();

#endif
