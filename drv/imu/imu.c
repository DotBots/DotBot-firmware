/**
 * @file imu.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for calculating the Euler angles.
 *
 * @copyright Inria, 2023
 *
 */

#include "math.h"
#include "imu.h"

float imu_calculate_roll(lsm6ds_acc_data_t *acc_reading) {
    // return roll
    return atan2f(-acc_reading->x, acc_reading->z);
}

float imu_calculate_pitch(lsm6ds_acc_data_t *acc_reading) {
    // first calculate roll
    float roll = imu_calculate_roll(acc_reading);
    // then calculate pitch
    float gz2 = (float)-acc_reading->x * sinf(roll) + (float)acc_reading->z * cosf(roll);
    return atanf((float)acc_reading->y / gz2);
}

float imu_calculate_uncompensated_heading(lis2mdl_compass_data_t *mag_reading) {
    // convert to heading
    // atan2(x,y) for north-clockwise convention, + Pi for 0 to 2PI heading
    return atan2f(mag_reading->x, mag_reading->y) + M_PI;
}

float imu_calculate_tilt_compensated_heading(lis2mdl_compass_data_t *mag_reading, lsm6ds_acc_data_t *acc_reading) {
    // Discard the sign of roll and pitch
    float roll  = fabsf(imu_calculate_roll(acc_reading));
    float pitch = fabsf(imu_calculate_pitch(acc_reading));

    float by2 = (float)mag_reading->z * sinf(roll) + (float)mag_reading->x * cosf(roll);
    float bz2 = (float)-mag_reading->x * sinf(roll) + mag_reading->z * cosf(roll);

    float bx3 = (float)mag_reading->y * cosf(pitch) + (float)bz2 * sinf(pitch);

    float ret = atan2f(by2, bx3) + M_PI;

    return ret;
}
