/**
 * @file 01bsp_lighthouse.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the lighthouse v2 chip in the DotBot board.
 *
 * Load this program on your board. LED should blink blue when it receives a valid lighthouse 2 signal.
 *
 * @date 2022
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "lh2.h"

//=========================== defines =========================================

//=========================== variables =========================================

static db_lh2_t _lh2;

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    db_lh2_init();

    // Start SPI capture
    db_lh2_start_transfer(&_lh2);

    while (1) {
        // the location function has to be running all the time but not that fast
        db_lh2_process_location(&_lh2);

        // wait until the packet is ready
        if (_lh2.state == DB_LH2_READY) {
            db_lh2_stop_transfer(&_lh2);
            db_lh2_start_transfer(&_lh2);
            __NOP();
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
