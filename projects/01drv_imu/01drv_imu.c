/**
 * @file 03app_sailbot.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is an example on how to use the IMU driver.
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

#include "imu.h"
#include "gpio.h"

//=========================== defines =========================================

#define CALIBRATION_PROCEDURE (0)
#define CONST_PI              (3.14159265359f)

typedef struct {
} drv_imu_vars_t;

//=========================== variables =========================================

drv_imu_vars_t _drv_imu_vars;

//=========================== prototypes =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the IMU
    imu_init(NULL);

#if CALIBRATION_PROCEDURE
    float offset_x;
    float offset_y;
    float offset_z;

    imu_magnetometer_calibrate(&offset_x, &offset_y, &offset_z);
    printf("offset_x: %f offset_y: %f offset_z: %f\n", offset_x, offset_y, offset_z);
    while (1) {
        ;
    }
#else
    float heading;

    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (imu_data_ready()) {
            imu_read_heading();
            heading                  = imu_last_heading() * 180 / CONST_PI;
            printf("heading: %f\n", heading);
        }
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
#endif
}

//=========================== functions =========================================
