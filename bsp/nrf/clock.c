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

//=========================== defines ==========================================

typedef struct {
    bool hf_enabled;  ///< Checks whether high frequency clock is running
    bool lf_enabled;  ///< Checks whether low frequency clock is running
} clock_state_t;

//=========================== variables ========================================

static clock_state_t _clock_state = {
    .hf_enabled = false,
    .lf_enabled = false,
};

//=========================== public ===========================================

void db_hfclk_init(void) {
    if (_clock_state.hf_enabled) {
        // Do nothing if already running
        return;
    }

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
    // Enable 128MHZ core clock, only possible with application core
    NRF_CLOCK->HFCLKCTRL = CLOCK_HFCLKCTRL_HCLK_Div1 << CLOCK_HFCLKCTRL_HCLK_Pos;
#endif

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;
    NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
    _clock_state.hf_enabled = true;
}

void db_lfclk_init(void) {
    if (_clock_state.lf_enabled) {
        // Do nothing if already running
        return;
    }

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
#if defined(NRF5340_XXAA)
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_LFRC << CLOCK_LFCLKSRC_SRC_Pos);
#else
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
#endif
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
    _clock_state.lf_enabled = true;
}
