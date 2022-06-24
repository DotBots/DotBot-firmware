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
// Inlcude BSP packages
#include <board.h>
#include <motors.h>


//=========================== defines =========================================

// Define a blocking wait function.
#define WAIT_A_BIT(PAUSE) for (int i = 0; i < 3000 * PAUSE; i++) {;}    ///< The 3000 magic number, approximates to about 1ms per 1 unit of PAUSE.

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Configure Motors
    db_motors_init();

    while (1) {
        // Right motor forward
        db_motors_set_speed(0, 60);
        WAIT_A_BIT(2000);            // wait ~2 sec
        db_motors_set_speed(0, 0);   // Turn off motor
        WAIT_A_BIT(1000);            // wait ~1 sec

        // Right motor backward
        db_motors_set_speed(0, -60);
        WAIT_A_BIT(2000);           // wait ~2 sec
        db_motors_set_speed(0, 0);  // Turn off motor
        WAIT_A_BIT(1000);           // wait ~1 sec

        // Left motor forward
        db_motors_set_speed(60, 0);
        WAIT_A_BIT(2000);           // wait ~2 sec
        db_motors_set_speed(0, 0);  // Turn off motor
        WAIT_A_BIT(1000);           // wait ~1 sec

        // Left motor backward
        db_motors_set_speed(-60, 0);
        WAIT_A_BIT(2000);           // wait ~2 sec
        db_motors_set_speed(0, 0);  // Turn off motor
        WAIT_A_BIT(1000);           // wait ~1 sec
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
