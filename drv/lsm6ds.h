#ifndef __LSM6DS_H
#define __LSM6DS_H

/**
 * @defgroup    drv_lsm6ds   LSM6DS IMU driver
 * @ingroup     drv
 * @brief       Driver for the ST LSM6DS IMU
 *
 * @{
 * @file
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

/// Accelerometer data
typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lsm6ds_acc_data_t;

typedef void (*lsm6ds_data_ready_cb_t)(void);  ///< Callback function prototype, it is called on each available sample

/**
 * @brief Initialize the LSM6DS chip
 *
 * @param[in] callback       callback pointer invoked whenever data is ready
 */
void lsm6ds_init(lsm6ds_data_ready_cb_t callback);

/**
 * @brief Checks whether LSM6DS data is ready for fetch
 */
bool lsm6ds_data_ready(void);

/**
 * @brief Reads accelerometer data on LSM6DS over I2C
 *
 * @param[out] out Struct to write data to
 */
void lsm6ds_read_accelerometer(lsm6ds_acc_data_t *out);

#endif
