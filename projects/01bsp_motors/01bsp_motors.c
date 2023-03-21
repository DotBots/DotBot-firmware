/**
 * @file 01bsp_motors.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the motor driver in the DotBot board.
 *
 * Load this program on your board. The right wheel should spin forwards and then backward.
 * Afterwards the left wheel also turns forward and then backward.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Include BSP packages
#include "board.h"
#include "motors.h"
#include "timer.h"

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Initialize the timer
    db_timer_init();

    // Configure Motors
    db_motors_init();

    while (1) {
        // Move forward
        for (uint8_t speed = 50; speed < 100; speed++) {
            db_motors_set_speed(speed, speed);
            db_timer_delay_ms(20);
        }

        // Move backward
        for (uint8_t speed = 50; speed < 100; speed++) {
            db_motors_set_speed(speed * -1, speed * -1);
            db_timer_delay_ms(20);
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
