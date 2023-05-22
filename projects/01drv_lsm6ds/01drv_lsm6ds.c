/**
 * @file 01drv_lsm6ds.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is an example on how to use the LSM6DS driver.
 *
 * Load this program on your board in debug mode, roll will be continuously
 * printed.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>

// Include BSP packages

#include "lsm6ds.h"

#define CONST_PI (3.14159265359f)

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the IMU
    lsm6ds_init(NULL);

    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (lsm6ds_data_ready()) {
            lsm6ds_read_accelerometer();
            int16_t roll  = (int16_t)(lsm6ds_last_roll() * 180 / CONST_PI);
            int16_t pitch = (int16_t)(lsm6ds_last_pitch() * 180 / CONST_PI);
            printf("roll: %d pitch = %d\n", roll, pitch);
        }
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
