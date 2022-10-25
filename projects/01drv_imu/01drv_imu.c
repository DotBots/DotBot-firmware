/**
 * @file 03app_sailbot.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is the radio-controlled SailBot app.
 *
 * Load this program on your board. Now the SailBot can be remotely controlled
 * from a nearby nRF52840-DK.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

// Include BSP packages

#include "imu.h"
#include "gpio.h"

//=========================== defines =========================================

#define CALIBRATION_PROCEDURE (0u)
#define CONST_PI              (3.14159265359f)

//=========================== variables =========================================

//=========================== prototypes =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the IMU
    imu_init();

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
            heading = imu_read_heading() * 180 / CONST_PI;
            printf("heading: %f\n", heading);
        }
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
#endif
}

//=========================== functions =========================================