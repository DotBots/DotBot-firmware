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

/**
 * @brief Turn ON the DotBot board.
 * 
 * Especifically turn on the Board Regulator
 * all the on board regulators ENABLE pins
 * are tied to the nRF pin P0.20
 * 
 */
void db_board_init(void) {

    // Turn ON the DotBot board regulator 
    NRF_P0->DIRSET = 1 << 20;       // set pin as output
    NRF_P0->OUTSET = 1 << 20;       // set pin HIGH
}

/**
 * @brief Turn ON the on-board regulator.  
 *
 */
void db_board_setReg_ON(void) {

    // Turn ON the DotBot board regulator
    NRF_P0->OUTSET = 1 << 20; // set pin HIGH
}

/**
 * @brief Turn OFF the on-board regulator.
 *
 */
void db_board_setReg_OFF(void) {

    // Turn OFF the DotBot board regulator
    NRF_P0->OUTCLR = 1 << 20; // set pin LOW
}
