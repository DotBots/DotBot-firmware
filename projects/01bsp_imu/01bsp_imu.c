/**
 * @file 01bsp_i2c.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the I2C api.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "timer_hf.h"
#include "ism330.h"

//=========================== defines ==========================================



//=========================== variables ========================================

ism330_acc_data_t acc_data;
ism330_gyro_data_t gyro_data;

    //=========================== main =============================================

    /**
     *  @brief The program starts executing here.
     */
int main(void) {

    db_board_init();
    db_ism330_init();
    db_timer_hf_init();

    db_timer_hf_delay_ms(500);
    // read accelerometer and gyroscope data in a loop
    while (1) {
        // Read Accelerometer data
        db_ism330_accel_read(&acc_data);
        db_timer_hf_delay_ms(250);
        __NOP();  // A place for a break_point

        // Read Gyroscope data
        db_ism330_gyro_read(&gyro_data);
        db_timer_hf_delay_ms(250);
        __NOP();  // A place for a break_point
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
