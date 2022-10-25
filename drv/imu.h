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

typedef void (*imu_data_ready_cb_t)(void);  ///< Callback function prototype, it is called on each available sample

void  imu_init(imu_data_ready_cb_t callback);
float imu_read_heading();
void  imu_magnetometer_calibrate(float *offset_x, float *offset_y, float *offset_z);

#endif
