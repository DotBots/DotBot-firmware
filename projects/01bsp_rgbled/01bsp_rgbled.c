/**
 * @file 01bsp_rgbled.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the RGB LED in the DotBot board.
 *
 * Load this program on your board. The LEDs should start blinking different colors.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "rgbled.h"

//=========================== defines =========================================

// Define a blocking wait function.
#define WAIT_A_BIT(PAUSE) for (int i = 0; i < 3000 * PAUSE; i++) {;}    ///< The 3000 magic number, approximates to about 1ms per 1 unit of PAUSE.

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_rgbled_init();

    /* Change RGB colors in a loop */
    while (1) {
        db_rgbled_set(255, 0, 0);
        WAIT_A_BIT(2000);
        db_rgbled_set(0, 255, 0);
        WAIT_A_BIT(2000);
        db_rgbled_set(0, 0, 255);
        WAIT_A_BIT(2000);
        db_rgbled_set(255, 255, 0);
        WAIT_A_BIT(2000);
        db_rgbled_set(0, 255, 255);
        WAIT_A_BIT(2000);
        db_rgbled_set(255, 0, 255);
        WAIT_A_BIT(2000);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
