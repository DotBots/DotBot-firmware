/**
 * @file clock.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "clock" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdbool.h>
#include "clock.h"

//=========================== variables ========================================

static bool _hfclock_enabled = false;

//=========================== public ===========================================

/**
 * @brief Initialize and start the High Frequency clock.
 */
void db_hfclk_init(void) {
    if (_hfclock_enabled) {
        // Do nothing if already running
        return;
    }

    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0x00;
    NRF_CLOCK->TASKS_HFCLKSTART     = 0x01;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    _hfclock_enabled = true;
}

/**
 * @brief Stop the High Frequency clock.
 */
void db_hfclk_deinit(void) {
    if (!_hfclock_enabled) {
        // Do nothing if already running
        return;
    }

    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
    _hfclock_enabled = false;
}

/**
 * @brief Initialize and start the Low Frequency clock.
 */
void db_lfclk_init(void) {
    if (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) {
        // Do nothing if already running
        return;
    }

    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
}

/**
 * @brief Stop the Low Frequency clock.
 */
void db_lfclk_deinit(void) {
    if (!(NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk)) {
        // Do nothing if not running
        return;
    }

    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}
