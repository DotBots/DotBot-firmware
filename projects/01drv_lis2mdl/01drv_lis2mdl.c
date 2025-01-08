/**
 * @file
 * @ingroup samples_drv
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is an example on how to use the LIS2MDL driver.
 *
 * Raw compass values are continously printed as debug output.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <nrf.h>

// Include BSP packages

#include "lis2mdl.h"
#include "lsm6ds.h"

//=========================== defines =========================================

typedef struct {
} drv_lis2mdl_vars_t;

//=========================== main ============================================

int main(void) {
    // Init the magnetometer
    lis2mdl_init(NULL);
    printf("X,Y,Z\n");
    while (1) {
        // processor idle until an interrupt occurs and is handled
        if (lis2mdl_data_ready()) {
            lis2mdl_compass_data_t mag;
            lis2mdl_read_magnetometer(&mag);
            printf("%d,%d,%d\n", mag.x, mag.y, mag.z);
        }
        __WFE();
    }
}
