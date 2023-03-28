#ifndef __ISM330_H
#define __ISM330_H

/**
 * @file ism300.h
 * @addtogroup DRV
 *
 * @brief  drv module for the ISM330DHCXTR IMU.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"

//=========================== defines ==========================================

// IMU address
#define ISM330_ADDRESS 0X6A

// Accelerometer Registers
#define ISM330_REG_OUTX_L_A 0x28
#define ISM330_REG_OUTX_H_A 0x29
#define ISM330_REG_OUTY_L_A 0x2A
#define ISM330_REG_OUTY_H_A 0x2B
#define ISM330_REG_OUTZ_L_A 0x2C
#define ISM330_REG_OUTZ_H_A 0x2D

// Gyroscope Registers
#define ISM330_REG_OUTX_L_G 0x22
#define ISM330_REG_OUTX_H_G 0x23
#define ISM330_REG_OUTY_L_G 0x24
#define ISM330_REG_OUTY_H_G 0x25
#define ISM330_REG_OUTZ_L_G 0x26
#define ISM330_REG_OUTZ_H_G 0x27

// Configuration Registers
#define ISM330_REG_CTRL1_XL 0x10
#define ISM330_REG_CTRL2_G  0x11
#define ISM330_REG_CTRL3_C  0x12
#define ISM330_REG_CTRL4_C  0x13
#define ISM330_REG_CTRL5_C  0x14
#define ISM330_REG_CTRL6_C  0x15
#define ISM330_REG_CTRL7_G  0x16
#define ISM330_REG_CTRL8_XL 0x17
#define ISM330_REG_CTRL9_XL 0x18
#define ISM330_REG_CTRL10_C 0x19

//=========================== variables ========================================

///! Data type to store the accelerometer data in [cm/s^2]
typedef struct {
    float x;  ///< X axis
    float y;  ///< Y axis
    float z;  ///< Z axis
} ism330_acc_data_t;

///! Data type to store the gyrometer data in [rad/s]
typedef struct {
    float x;  ///< X axis
    float y;  ///< Y axis
    float z;  ///< Z axis
} ism330_gyro_data_t;

//=========================== prototypes ==========================================

/**
 * @brief Initialize the IMU, configures the data rate and range for all the sensors.
 *        Also activates  the High-Pass filter for the Gyro.
 *
 * @param[in]   sda  pointer to gpio serial data pin
 * @param[in]   scl  pointer to gpio serial clock pin
 */
void db_ism330_init(const gpio_t *sda, const gpio_t *sck);

/**
 * @brief Read the X, Y and Z values for the accelerometer.
 *        returned as a vector of 3 floats [x, y, z], in cm/s^2
 *
 * @param[out] data       Output vector of the accelerometer
 */
void db_ism330_accel_read(ism330_acc_data_t *data);

/**
 * @brief Read the X, Y and Z values for the gyroscope.
 *        returned as a vector of 3 floats [x, y, z], in radians/s
 *
 * @param[out] data       Output vector of the gyroscope
 */
void db_ism330_gyro_read(ism330_gyro_data_t *data);

#endif
