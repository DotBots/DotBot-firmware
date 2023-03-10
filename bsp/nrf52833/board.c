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
#include "gpio.h"

//=========================== defines =========================================

//=========================== variables =========================================

static const gpio_t _reg_pin = { .pin = 20, .port = 0 };

//=========================== public ==========================================

void db_board_init(void) {

    // Turn ON the DotBot board regulator
    db_gpio_init(&_reg_pin, DB_GPIO_OUT);
    db_gpio_set(&_reg_pin);
}

void db_board_regulator_on(void) {

    // Turn ON the DotBot board regulator
    db_gpio_set(&_reg_pin);
}

void db_board_regulator_off(void) {

    // Turn OFF the DotBot board regulator
    db_gpio_clear(&_reg_pin);
}
