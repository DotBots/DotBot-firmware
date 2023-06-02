#ifndef __IMU_H
#define __IMU_H

/**
 * @file imu.h
 * @addtogroup sailbot
 *
 * @brief  Module for calculating the Euler angles (roll, pitch, yaw) of the SailBot.
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include "lis2mdl.h"
#include "lsm6ds.h"

/**
 * @brief Calculate roll angle
 *
 * @param[in] acc_reading Raw accelerometer reading struct
 * @return Roll angle
 */
float imu_calculate_roll(lsm6ds_acc_data_t *acc_reading);

/**
 * @brief Calculate pitch angle
 *
 * @param[in] acc_reading Raw accelerometer reading struct
 * @return Pitch angle
 */
float imu_calculate_pitch(lsm6ds_acc_data_t *acc_reading);

/**
 * @brief Calculate uncompensated heading
 *
 * @param[in] mag_reading Raw magnetometer reading struct
 * @return Heading angle
 */
float imu_calculate_uncompensated_heading(lis2mdl_compass_data_t *mag_reading);

/**
 * @brief Calculate tilt-compensated heading
 *
 * @param[in] mag_reading Raw magnetometer reading struct
 * @param[in] acc_reading Raw accelerometer reading struct
 * @return Tilt-compensated heading angle
 */
float imu_calculate_tilt_compensated_heading(lis2mdl_compass_data_t *mag_reading, lsm6ds_acc_data_t *acc_reading);

#endif