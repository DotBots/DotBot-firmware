/**
 * @file
 * @ingroup samples_bsp
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
#include "board_config.h"
#include "lh2_4.h"

//=========================== defines ==========================================

#define DB2_LH2_4_FULL_COMPUTATION 1

//=========================== variables ========================================

static db_lh2_4_t _lh2;

//=========================== main =============================================

/*
P0.30 -> TS4231 Data pin
P1.04 -> SPI SCLK
P0.29 ->    High -> at the start of _reverse_p_count()
            Low  - at the end of _reverse_p_count()
P1.07 -> HIGH during the LSFR update in _reverse_p_count()
P0.28 -> HIGH during the checkpoint check in _reverse_p_count().
P1.11 -> Goes high if the original function and the function with the changes being tested gives differet results

*/


/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    db_lh2_4_init(&_lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_4_start(&_lh2);
    NRF_P0->DIRSET = 1 << 29;
    NRF_P0->DIRSET = 1 << 28;
    NRF_P1->DIRSET = 1 << 07;
    NRF_P1->DIRSET = 1 << 11;


    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();
        db_lh2_4_process_raw_data(&_lh2);

        if (DB2_LH2_4_FULL_COMPUTATION) {
            // the location function has to be running all the time
            db_lh2_4_process_location(&_lh2);
            __NOP();
            }

            //db_lh2_4_start(&_lh2);
  
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
// NRF_P0->OUTSET = 1 << 29;
// NRF_P0->OUTCLR = 1 << 29;