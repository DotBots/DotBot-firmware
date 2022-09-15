/**
 * @file board.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "board" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "board.h"

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== public ==========================================

void db_board_init(void) {

    // Turn ON the DotBot board regulator
    NRF_P0->DIRSET = 1 << 20;  // set pin as output
    NRF_P0->OUTSET = 1 << 20;  // set pin HIGH
}

void db_board_regulator_on(void) {

    // Turn ON the DotBot board regulator
    NRF_P0->OUTSET = 1 << 20;  // set pin HIGH
}

void db_board_regulator_off(void) {

    // Turn OFF the DotBot board regulator
    NRF_P0->OUTCLR = 1 << 20;  // set pin LOW
}
