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

bool      packet_ready;
uint32_t *current_loc_p;
//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    lh2_init();

    // Start SPI capture
    NRF_P0->DIRSET = 1 << 20;
    NRF_P0->OUTSET = 1 << 20;
    start_transfer();

    while (1) {
        // the location function has to be running all the time but not that fast
        packet_ready = get_black_magic();

        // wait until the packet is ready
        if (packet_ready) {

            NRF_P0->OUTSET = 1 << 20;
            current_loc_p  = get_current_location();
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
