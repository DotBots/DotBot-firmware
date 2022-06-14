/**
 * @file 01bsp_rpm.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the Wheel revolution counter in the DotBot board.
 *
 * Load this program on your board. The LEDs should start blinking blue as the wheels are being turned.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "rpm.h"


//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    puts("Revolution counter application");
    db_board_init();
    db_revolution_counter_init();
    db_board_encoder_timers_start();

    uint32_t left_speed, right_speed, left_rpm, right_rpm, left_rps, right_rps = 0;
    while (1) {
        left_speed = db_board_get_left_speed();
        left_rpm = db_board_get_left_rpm();
        left_rps = db_board_get_left_rps();
        right_speed = db_board_get_right_speed();
        right_rpm = db_board_get_right_rpm();
        right_rps = db_board_get_right_rps();
        printf("Left  - speed: %i, RPM: %i, RPS: %i\n", left_speed, left_rpm, left_rps);
        printf("Right - speed: %i, RPM: %i, RPS: %i\n", right_speed, right_rpm, right_rps);
        uint32_t wait = 0x00fffff;
        while (wait--) {}
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
