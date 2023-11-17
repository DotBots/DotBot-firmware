/**
 * @file
 * @ingroup samples_bsp
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
#include "timer.h"

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_timer_init();
    db_rgbled_init();

    /* Change RGB colors in a loop */
    while (1) {
        db_rgbled_set(255, 0, 0);
        db_timer_delay_s(2);
        db_rgbled_set(0, 255, 0);
        db_timer_delay_s(2);
        db_rgbled_set(0, 0, 255);
        db_timer_delay_s(2);
        db_rgbled_set(255, 255, 0);
        db_timer_delay_s(2);
        db_rgbled_set(0, 255, 255);
        db_timer_delay_s(2);
        db_rgbled_set(255, 0, 255);
        db_timer_delay_s(2);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
