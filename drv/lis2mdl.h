#ifndef __LIS2MDL_H
#define __LIS2MDL_H

/**
 * @file lis2mdl.h
 * @addtogroup sailbot
 *
 * @brief  Module for reading the LIS2MDL magnetometer.
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
} lis2mdl_compass_data_t;

typedef void (*lis2mdl_data_ready_cb_t)(void);  ///< Callback function prototype, it is called on each available sample

/**
 * @brief Initialize the LIS2MDL chip
 *
 * @param[in] callback       callback pointer invoked whenever data is ready
 */
void lis2mdl_init(lis2mdl_data_ready_cb_t callback);

/**
 * @brief Checks whether LIS2MDL data is ready for fetch
 */
bool lis2mdl_data_ready(void);

/**
 * @brief Reads magnetometer data on LIS2MDL over I2C
 *
 * @param[out] out Struct to write data to
 */
void lis2mdl_read_magnetometer(lis2mdl_compass_data_t *out);

#endif
