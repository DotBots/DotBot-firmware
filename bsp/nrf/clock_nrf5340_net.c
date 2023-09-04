/**
 * @file clock_nrf5340_net.c
 * @addtogroup BSP
 *
 * @brief  nrf5340-net-specific definition of the "clock" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdbool.h>
#include "clock.h"

static bool _hf_enabled = false;

//=========================== public ===========================================

void db_hfclk_init(void) {
    if (_hf_enabled) {
        // Do nothing if already running
        return;
    }

    NRF_CLOCK_NS->EVENTS_HFCLKSTARTED = 0;
    while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 1) {}

    NRF_CLOCK_NS->HFCLKSRC         = CLOCK_HFCLKSRC_SRC_HFXO << CLOCK_HFCLKSRC_SRC_Pos;
    NRF_CLOCK_NS->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK_NS->EVENTS_HFCLKSTARTED == 0) {}
    _hf_enabled = true;
}

void db_lfclk_init(void) {
}
