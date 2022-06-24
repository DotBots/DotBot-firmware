#ifndef __BOARD_H
#define __BOARD_H

/**
 * @file board.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "board" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

//=========================== defines ==========================================

/**
 * Enable a pin as output, must be called once before using the other macros below
 */
#define DB_PIN_ENABLE(port, pin)    NRF_P##port->DIRSET = (1 << pin)
#define DB_PIN_ON(port, pin)        NRF_P##port->OUTSET = (1 << pin)    /**< Turn on the pin */
#define DB_PIN_OFF(port, pin)       NRF_P##port->OUTCLR = (1 << pin)    /**< Turn off the pin */
#define DB_PIN_TOGGLE(port, pin)    NRF_P##port->OUT ^= (1 << pin)      /**< Toggle the pin */

//=========================== public ===========================================

void db_board_init(void);
void db_board_regulator_on(void);
void db_board_regulator_off(void);

#endif
