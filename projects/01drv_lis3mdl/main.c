/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an example on how to use the LIS3MDL driver.
 *
 * @copyright Inria, 2023
 *
 */
#include <stdint.h>
#include <stdio.h>

#include <nrf.h>

#include "board_config.h"
#include "board.h"
#include "gpio.h"
#include "timer.h"
#include "lis3mdl.h"

#define TIMER_DEV                (0)
#define LIS3MDL_READ_INTERVAL_MS (100U)

static const gpio_t mag_drdy = {
    .port = DB_LIS3MDL_DRDY_PORT,
    .pin  = DB_LIS3MDL_DRDY_PIN,
};

static const lis3mdl_conf_t _lis3mdl_conf = {
    .scl      = &db_scl,
    .sda      = &db_sda,
    .mag_drdy = &mag_drdy,
    .xy_mode  = LIS3MDL_XY_MODE_MEDIUM,
    .z_mode   = LIS3MDL_Z_MODE_MEDIUM,
    .odr      = LIS3MDL_ODR_20Hz,
    .op_mode  = LIS3MDL_OP_CONTINUOUS,
    .scale    = LIS3MDL_SCALE_4G,
};

int main(void) {
    db_board_init();
    db_timer_init(TIMER_DEV);
    lis3mdl_init(&_lis3mdl_conf);
    puts("X,Y,Z");
    while (1) {
        if (lis3mdl_data_ready()) {
            lis3mdl_data_t mag;
            lis3mdl_read_magnetometer(&mag);
            printf("%d,%d,%d\n", mag.x, mag.y, mag.z);
        }
        int16_t temperature = 0;
        lis3mdl_read_temperature(&temperature);
        printf("T: %iC\n", temperature);
        db_timer_delay_ms(TIMER_DEV, LIS3MDL_READ_INTERVAL_MS);
    }
}
