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

#include "lis2mdl.h"
#include "gpio.h"

//=========================== defines =========================================

#define CALIBRATION_PROCEDURE (0)
#define CONST_PI              (3.14159265359f)

typedef struct {
} drv_lis2mdl_vars_t;

//=========================== variables =========================================

static drv_lis2mdl_vars_t _drv_lis2mdl_vars;

//=========================== prototypes =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the IMU
    lis2mdl_init(NULL);

#if CALIBRATION_PROCEDURE
    lis2mdl_compass_data_t offset;

    lis2mdl_magnetometer_calibrate(&offset);
    while (1) {
        ;
    }
#else
    float heading;

    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (lis2mdl_data_ready()) {
            lis2mdl_read_heading();
            heading = lis2mdl_last_heading() * 180 / CONST_PI;
            printf("heading: %f\n", heading);
        }
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
#endif
}

//=========================== functions =========================================
