#ifndef __LSM6DS_H
#define __LSM6DS_H

/**
 * @file lsm6ds.h
 * @addtogroup sailbot
 *
 * @brief  Driver for LSM6DS 6D IMU.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <nrf.h>

typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lsm6ds_acc_data_t;

typedef void (*lsm6ds_data_ready_cb_t)(void);  ///< Callback function prototype, it is called on each available sample

void   lsm6ds_init(lsm6ds_data_ready_cb_t callback);
bool   lsm6ds_data_ready(void);
void   lsm6ds_read_accelerometer(void);
int8_t lsm6ds_last_roll(void);

#endif
