/**
 * @file i2c.c
 * @addtogroup BSP
 *
 * @brief  bsp module for the ISM330DHCXTR IMU.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <nrf.h>
#include "gpio.h"
#include "i2c.h"
#include "ism330.h"
#include "math.h"

//=========================== public functions ================================

void db_ism330_init(void)
{
    // Initialize I2C
    db_i2c_init(&_ism330_scl_gpio, &_ism330_sda_gpio);

        // Read WHOAMI register to verify that the system works

    // Configure the accelerometer
    uint8_t ism330_CTRL1_XL_val = 0b10001000;      // ODR_XL[7:4] = 0b1000 -> 1.66kHz Accelerometer Data Rate.
                                                   // FS_XL [3:2] = 0b10   -> +- 4g   Accelerometer Range
                                                   // LPF2_XL_EN[1] = 0b0  -> Low Pass Filter 2 disabled.
    db_i2c_begin();
    db_i2c_write_regs(ISM330_ADDRESS, ISM330_REG_CTRL1_XL, &ism330_CTRL1_XL_val, 1);
    db_i2c_end();


    // Configure the Gyroscope
    // Data rate and range
    uint8_t ism330_CTRL2_G_val = 0b10000100;   // ODR_G[7:4] = 0b1000 -> 1.66kHz Gyroscope Data Rate.
                                               // FS_G [3:2] = 0b01   -> +-500dps   Gyroscope Range                        
    db_i2c_begin();
    db_i2c_write_regs(ISM330_ADDRESS, ISM330_REG_CTRL2_G, &ism330_CTRL2_G_val, 1);
    db_i2c_end();

    // High Pass filter for the Gyroscope
    uint8_t ism330_CTRL7_G_val = 0b01010000;  // G_HM_MODE[7]   = 0b0  -> Gyro high power mode enabled
                                              // HP_EN_G  [6]   = 0b1  -> Gyro high-pass filter enabled
                                              // HPM_G    [5:4] = 0b01 -> high-pass filter cut off frequency = 65mHz
    db_i2c_begin();
    db_i2c_write_regs(ISM330_ADDRESS, ISM330_REG_CTRL7_G, &ism330_CTRL7_G_val, 1);
    db_i2c_end();
}

void db_ism330_accel_read(ism330_acc_data_t *data) 
{
    uint8_t tmp[6];
    int16_t acc_x, acc_y, acc_z;

    // Read all the acceleration ouput registers in one swift operation.
    db_i2c_begin();
    db_i2c_read_regs(ISM330_ADDRESS, ISM330_REG_OUTX_L_A, &tmp, 6);
    db_i2c_end();

    // The values from the IMU are split in 2 variables per axis, join them together
    acc_x = tmp[0] | (uint16_t)(tmp[1] << 8);
    acc_y = tmp[2] | (uint16_t)(tmp[3] << 8);
    acc_z = tmp[4] | (uint16_t)(tmp[5] << 8);

    // Scale the values to cm/s^2
    data->x = -1 * acc_x * 0.122 / 1000.0 * 980.665;  //  acc_x * (mG/LSB) * (G/mG) * ((cm/s^2) / G)
    data->y = -1 * acc_y * 0.122 / 1000.0 * 980.665;
    data->z = -1 * acc_z * 0.122 / 1000.0 * 980.665;
}

void db_ism330_gyro_read(ism330_gyro_data_t *data) 
{
    uint8_t tmp[6];
    int16_t gyro_x, gyro_y, gyro_z;

    // Read all the gyroeleration ouput registers in one swift operation.
    db_i2c_begin();
    db_i2c_read_regs(ISM330_ADDRESS, ISM330_REG_OUTX_L_G, &tmp, 6);
    db_i2c_end();

    // The values from the IMU are split in 2 variables per axis, join them together
    gyro_x = tmp[0] | (uint16_t)(tmp[1] << 8);
    gyro_y = tmp[2] | (uint16_t)(tmp[3] << 8);
    gyro_z = tmp[4] | (uint16_t)(tmp[5] << 8);

    // Scale the values to cm/s^2
    data->x = gyro_x * 17.5 / 1000.0 * M_PI / 180;  //  gyro_x * (mdps/LSB) * (dps/mpds) * (radps/dps)
    data->y = gyro_y * 17.5 / 1000.0 * M_PI / 180;
    data->z = gyro_z * 17.5 / 1000.0 * M_PI / 180;
}