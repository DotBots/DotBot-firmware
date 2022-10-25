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
#include <stdbool.h>

// Include BSP packages

#include "imu.h"
#include "gpio.h"

//=========================== defines =========================================

#define CALIBRATION_PROCEDURE (1)
#define CONST_PI              (3.14159265359f)

typedef struct {
    bool data_ready;
} drv_imu_vars_t;

//=========================== variables =========================================

drv_imu_vars_t _drv_imu_vars;

//=========================== prototypes =========================================

static void imu_data_ready_cb(void);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the IMU
    imu_init(&imu_data_ready_cb);

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
        if (_drv_imu_vars.data_ready) {
            heading = imu_read_heading() * 180 / CONST_PI;
            _drv_imu_vars.data_ready = false;
            printf("heading: %f\n", heading);
        }
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
#endif
}

//=========================== functions =========================================

static void imu_data_ready_cb(void) {
    _drv_imu_vars.data_ready = true;
}