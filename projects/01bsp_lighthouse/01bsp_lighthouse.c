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

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);

        // Reset the LH2 driver if a packet is ready. At this point, locations
        // can be read from lh2.results array
        if (_lh2.state == DB_LH2_READY) {
            db_lh2_reset(&_lh2);
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
