/**
 * @file
 * @ingroup bsp_board
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
#include "board_config.h"
#include "gpio.h"

//=========================== defines =========================================

//=========================== variables =========================================

#if defined(DB_REGULATOR_PORT)
static const gpio_t _reg_pin = { .port = DB_REGULATOR_PORT, .pin = DB_REGULATOR_PIN };
#endif

//=========================== public ==========================================

void db_board_init(void) {

#if defined(DB_REGULATOR_PORT)
    // Turn ON the DotBot board regulator if provided
    db_gpio_init(&_reg_pin, DB_GPIO_OUT);
    db_gpio_set(&_reg_pin);
#endif
}

// TODO: add a 3v minimote UICR code
