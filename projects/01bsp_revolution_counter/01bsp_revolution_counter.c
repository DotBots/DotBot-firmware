/**
 * @file 01bsp_revolution_counter.c
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
#include "revolution_counter.h"


//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_revolution_counter_init();
    db_board_encoder_timers_start();

    float left_speed, right_speed = 0.0;
    uint32_t left_rpm, right_rpm = 0;
    while (1) {
      uint32_t wait = 0x00fffff;
      while (wait--) {}
      left_speed = db_board_get_left_speed();
      left_rpm = db_board_get_left_rpm();
      right_speed = db_board_get_right_speed();
      right_rpm = db_board_get_right_rpm();
      __NOP();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
