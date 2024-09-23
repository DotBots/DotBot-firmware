/**
 * @file
 * @ingroup bsp_rng
 *
 * @brief  nRF52833-specific definition of the "rng" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <nrf.h>
#include <stdint.h>

#include "rng.h"

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define NRF_RNG NRF_RNG_NS
#endif

//=========================== public ===========================================

void db_rng_init(void) {

    // Enable bias correction, slower but uniform
    NRF_RNG->CONFIG = (RNG_CONFIG_DERCEN_Enabled << RNG_CONFIG_DERCEN_Pos);

    // Enable value ready event
    NRF_RNG->INTENSET = (RNG_INTENSET_VALRDY_Enabled << RNG_INTENSET_VALRDY_Pos);

    // Automatically stop when value is ready
    NRF_RNG->SHORTS = (RNG_SHORTS_VALRDY_STOP_Enabled << RNG_SHORTS_VALRDY_STOP_Pos);
}

void db_rng_read(uint8_t *value) {
    NRF_RNG->TASKS_START = 1;
    while (NRF_RNG->EVENTS_VALRDY == 0) {};
    *value                 = (uint8_t)NRF_RNG->VALUE;
    NRF_RNG->EVENTS_VALRDY = 0;
}
