/**
 * @file
 * @ingroup samples_drv
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is an example on how to use the IMU driver.
 *
 * @copyright Inria, 2023
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <nrf.h>
#include <stdbool.h>

// Include BSP packages
#include "imu.h"
#include "lis2mdl.h"
#include "lsm6ds.h"

//=========================== defines =========================================

#define CONST_PI (3.14159265359f)

typedef struct {
} drv_imu_vars_t;

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    lis2mdl_compass_data_t mag = { 0 };
    lsm6ds_acc_data_t      acc = { 0 };

    // Init the IMU chips
    imu_init(NULL, NULL);

    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (lis2mdl_data_ready()) {
            lis2mdl_read_magnetometer(&mag);
        }
        if (lsm6ds_data_ready()) {
            lsm6ds_read_accelerometer(&acc);

            float roll  = imu_calculate_roll(&acc);
            float pitch = imu_calculate_pitch(&acc);

            int16_t compensated_heading   = (int16_t)(imu_calculate_tilt_compensated_heading(&mag, &acc) * 180.0 / CONST_PI);
            int16_t uncompensated_heading = (int16_t)(imu_calculate_uncompensated_heading(&mag) * 180.0 / CONST_PI);
            printf("uncompensated=%d compensated=%d roll=%d pitch=%d\n", uncompensated_heading, compensated_heading, (int16_t)(roll * 180.0 / CONST_PI), (int16_t)(pitch * 180.0 / CONST_PI));
        }
        __WFE();
    }
}
