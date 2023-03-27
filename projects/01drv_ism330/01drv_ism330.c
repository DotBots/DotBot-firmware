/**
 * @file 01drv_ism330.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to use the ISM330 IMU available on the DotBot
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdlib.h>
#include "board.h"
#include "timer_hf.h"
#include "ism330.h"

//=========================== defines ==========================================

//=========================== variables ========================================

static ism330_acc_data_t  acc_data;
static ism330_gyro_data_t gyro_data;

///! IMU SDA pin
static const gpio_t _ism330_sda_gpio = {
    .port = 0,
    .pin  = 10,
};

///! IMU SCL pin
static const gpio_t _ism330_scl_gpio = {
    .port = 0,
    .pin  = 9,
};

//=========================== main =============================================

int main(void) {

    db_board_init();
    db_ism330_init(&_ism330_sda_gpio, &_ism330_scl_gpio);
    db_timer_hf_init();

    db_timer_hf_delay_ms(500);
    // read accelerometer and gyroscope data in a loop
    while (1) {
        // Read Accelerometer data
        db_ism330_accel_read(&acc_data);
        db_timer_hf_delay_ms(250);
        __NOP();  // A place for a breakpoint

        // Read Gyroscope data
        db_ism330_gyro_read(&gyro_data);
        db_timer_hf_delay_ms(250);
        __NOP();  // A place for a breakpoint
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
