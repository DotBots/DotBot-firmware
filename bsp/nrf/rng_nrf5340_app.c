/**
 * @file rng_nrf5340_app.c
 * @addtogroup BSP
 *
 * @brief  nrf5340-app-specific definition of the "rng" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <nrf.h>
#include <stdint.h>

#include "rng.h"

//=========================== public ===========================================

void db_rng_init(void) {
}

void db_rng_read(uint8_t *value) {
    *value = 0;
}
