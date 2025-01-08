#ifndef __ISM330_H
#define __ISM330_H

/**
 * @defgroup    drv_ism330      ISM330 IMU driver
 * @ingroup     drv
 * @brief       Driver for the ST ISM330 IMU
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <nrf.h>
#include "gpio.h"

//=========================== defines ==========================================

/// IMU address
#define ISM330_ADDRESS 0X6A

/**
 * @brief   Accelerometer Registers
 * @{
 */
#define ISM330_REG_OUTX_L_A 0x28  ///< Accelerometer OUTX Low
#define ISM330_REG_OUTX_H_A 0x29  ///< Accelerometer OUTX High
#define ISM330_REG_OUTY_L_A 0x2A  ///< Accelerometer OUTY Low
#define ISM330_REG_OUTY_H_A 0x2B  ///< Accelerometer OUTY High
#define ISM330_REG_OUTZ_L_A 0x2C  ///< Accelerometer OUTZ Low
#define ISM330_REG_OUTZ_H_A 0x2D  ///< Accelerometer OUTZ High
/** @} */

/**
 * @brief   Gyroscope Registers
 * @{
 */
#define ISM330_REG_OUTX_L_G 0x22  ///< Gyroscope OUTX Low
#define ISM330_REG_OUTX_H_G 0x23  ///< Gyroscope OUTX High
#define ISM330_REG_OUTY_L_G 0x24  ///< Gyroscope OUTY Low
#define ISM330_REG_OUTY_H_G 0x25  ///< Gyroscope OUTY High
#define ISM330_REG_OUTZ_L_G 0x26  ///< Gyroscope OUTZ Low
#define ISM330_REG_OUTZ_H_G 0x27  ///< Gyroscope OUTZ High
/** @} */

/**
 * @brief   Configuration Registers
 * @{
 */
#define ISM330_REG_CTRL1_XL 0x10  ///< Control Register 1
#define ISM330_REG_CTRL2_G  0x11  ///< Control Register 2
#define ISM330_REG_CTRL3_C  0x12  ///< Control Register 3
#define ISM330_REG_CTRL4_C  0x13  ///< Control Register 4
#define ISM330_REG_CTRL5_C  0x14  ///< Control Register 5
#define ISM330_REG_CTRL6_C  0x15  ///< Control Register 6
#define ISM330_REG_CTRL7_G  0x16  ///< Control Register 7
#define ISM330_REG_CTRL8_XL 0x17  ///< Control Register 8
#define ISM330_REG_CTRL9_XL 0x18  ///< Control Register 9
#define ISM330_REG_CTRL10_C 0x19  ///< Control Register 10
/** @} */

//=========================== variables ========================================

/// Data type to store the accelerometer data in [cm/s^2]
typedef struct {
    float x;  ///< X axis
    float y;  ///< Y axis
    float z;  ///< Z axis
} ism330_acc_data_t;

/// Data type to store the gyrometer data in [rad/s]
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
void db_ism330_init(const gpio_t *sda, const gpio_t *scl);

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
