#ifndef __LIS3MDL_H
#define __LIS3MDL_H

/**
 * @defgroup    drv_lis3mdl     LIS3MDL magnetometer driver
 * @ingroup     drv
 * @brief       Driver for the ST LIS3MDL magnetometer
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

#include "gpio.h"

/// Magnetometer data
typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lis3mdl_data_t;

/// XY mode
typedef enum {
    LIS3MDL_XY_MODE_LOW    = 0x00,  ///< Low-power
    LIS3MDL_XY_MODE_MEDIUM = 0x20,  ///< Medium-performance
    LIS3MDL_XY_MODE_HIGH   = 0x40,  ///< High-performance
    LIS3MDL_XY_MODE_ULTRA  = 0x60,  ///< Ultra-High-performance
} lis3mdl_xy_mode_t;

/// Z mode
typedef enum {
    LIS3MDL_Z_MODE_LOW    = 0x00,  ///< Low-power
    LIS3MDL_Z_MODE_MEDIUM = 0x04,  ///< Medium-performance
    LIS3MDL_Z_MODE_HIGH   = 0x08,  ///< High-performance
    LIS3MDL_Z_MODE_ULTRA  = 0x0C,  ///< Ultra-High-performance
} lis3mdl_z_mode_t;

/// Output data rate
typedef enum {
    LIS3MDL_ODR_0_625Hz = 0x00,  ///< 0.625Hz
    LIS3MDL_ODR_1_25Hz  = 0x04,  ///< 1.250Hz
    LIS3MDL_ODR_2_5Hz   = 0x08,  ///< 5Hz
    LIS3MDL_ODR_10Hz    = 0x10,  ///< 10Hz
    LIS3MDL_ODR_20Hz    = 0x14,  ///< 20Hz
    LIS3MDL_ODR_40Hz    = 0x18,  ///< 40Hz
    LIS3MDL_ODR_80Hz    = 0x1C,  ///< 80Hz
} lis3mdl_odr_t;

/// Operating mode
typedef enum {
    LIS3MDL_OP_CONTINUOUS = 0x00,  ///< Continous-conversion
    LIS3MDL_OP_SINGLE     = 0x01,  ///< Single-conversion
    LIS3MDL_OP_POWER_DOWN = 0x11,  ///< Power-down
} lis3mdl_op_t;

/// Scale
typedef enum {
    LIS3MDL_SCALE_4G  = 0x00 << 4,  ///< +-4 gauss
    LIS3MDL_SCALE_8G  = 0x01 << 4,  ///< +-8 gauss
    LIS3MDL_SCALE_12G = 0x10 << 4,  ///< +-12 gauss
    LIS3MDL_SCALE_16G = 0x11 << 4,  ///< +-16 gauss
} lis3mdl_scale_t;

/// LIS3MDL init configuration
typedef struct {
    const gpio_t     *scl;       ///< SCL gpio
    const gpio_t     *sda;       ///< SDA gpio
    const gpio_t     *mag_drdy;  ///< Data ready gpio
    lis3mdl_xy_mode_t xy_mode;   ///< XY mode
    lis3mdl_z_mode_t  z_mode;    ///< Z mode
    lis3mdl_odr_t     odr;       ///< Output data rate
    lis3mdl_op_t      op_mode;   ///< Operation mode
    lis3mdl_scale_t   scale;     ///< Scale
} lis3mdl_conf_t;

/**
 * @brief Initialize the LIS3MDL chip
 *
 * @param[in] conf          pointer to the init configuration
 */
void lis3mdl_init(const lis3mdl_conf_t *conf);

/**
 * @brief Checks whether LIS3MDL data is ready for fetch
 */
bool lis3mdl_data_ready(void);

/**
 * @brief Reads magnetometer data on LIS3MDL over I2C
 *
 * Should be called if lis3mdl_data_ready returns true
 *
 * @param[out] out Struct to write data to
 */
void lis3mdl_read_magnetometer(lis3mdl_data_t *out);

/**
 * @brief Reads LIS3MDL temperature sensor
 *
 * @param[out] temperature
 */
void lis3mdl_read_temperature(int16_t *temperature);

#endif
