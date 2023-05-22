/**
 * @file 01drv_lis2mdl.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is an example on how to use the LIS2MDL driver.
 *
 * Load this program on your board, set CALIBRATION_PROCEDURE to 1 if calibration is needed.
 * With CALIBRATION_PROCEDURE set to 0, compass heading is printed as debug output.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>

// Include BSP packages

#include "lis2mdl.h"
#include "lsm6ds.h"

//=========================== defines =========================================

#define CALIBRATION_PROCEDURE (0)
#define CONST_PI              (3.14159265359f)

typedef struct {
} drv_lis2mdl_vars_t;

//=========================== variables =========================================

//=========================== prototypes =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the magnetometer
    lis2mdl_init(NULL);

    // Init the IMU to read pitch and roll values to compensate heading
    lsm6ds_init(NULL);

#if CALIBRATION_PROCEDURE
    lis2mdl_compass_data_t offset;

    lis2mdl_magnetometer_calibrate(&offset);
    while (1) {
        ;
    }
#else

    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (lis2mdl_data_ready()) {
            lis2mdl_read_heading();
        }
        if (lsm6ds_data_ready()) {
            lsm6ds_read_accelerometer();
            float roll = lsm6ds_last_roll();
            float pitch = lsm6ds_last_pitch();
            int16_t compensated_heading = (int16_t) (lis2mdl_last_tilt_compensated_heading(roll, pitch) * 180.0 / CONST_PI);
            int16_t uncompensated_heading = (int16_t) (lis2mdl_last_uncompensated_heading() * 180.0 / CONST_PI);
            printf("uncompensated=%d compensated=%d roll=%d pitch=%d\n", uncompensated_heading, compensated_heading, (int16_t) (roll * 180.0 / CONST_PI), (int16_t) (pitch * 180.0/CONST_PI));
        }
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
#endif
}

//=========================== functions =========================================
