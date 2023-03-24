#ifndef __ISM330_H
#define __ISM330_H

/**
 * @file ism300.h
 * @addtogroup BSP
 *
 * @brief  bsp module for the ISM330DHCXTR IMU.
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

///! IMU SDA pin
static const gpio_t _ism330_sda_gpio = {
    .port = 0,
    .pin  = 10,
};

///! IMU SCL pin
static const gpio_t _ism330_scl_gpio = {
    .port = 0,
    .pin  = 9,
};

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
 *
 */
void db_ism330_init(void);

// /**
//  * @brief Configure
//  * 
//  * @param[in] range           Range of the data for the accelerometer. Accepted values +-(2,4,8,16) g
//  * @param[in] data_rate       pointer to struct that represents the SDA pin
//  */
// void db_ism330_accel_config(int8_t range, int16_t data_rate);

/**
 * @brief Read the X, Y and Z values for the accelerometer.
 *        returned as a vector of 3 floats [x, y, z], in cm/s^2
 *
 * @param[out] data       Output vector of the accelerometer 
 */
void db_ism330_accel_read(ism330_acc_data_t *data);

/**
 * @brief Begin transmission on I2C
 *
 * @param[in] range           Range of the data for the gyroscapo. Accepted values +-(125,250,500,1000,2000,4000) dps
 * @param[in] data_rate       pointer to struct that represents the SDA pin
 */
// void db_ism330_gyro_config(int8_t range, int16_t data_rate);

/**
 * @brief Read the X, Y and Z values for the gyroscope.
 *        returned as a vector of 3 floats [x, y, z], in radians/s
 *
 * @param[out] data       Output vector of the gyroscope
 */
void db_ism330_gyro_read(ism330_gyro_data_t *data);

#endif