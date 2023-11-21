/**
 * @file
 * @ingroup bsp_clock
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

#ifndef NRF53_XOSC32_CAPACITANCE
#ifdef BOARD_DOTBOT_V2
#define NRF53_XOSC32_CAPACITANCE_X2 22  ///< 11pF on BT40 module
#else
#define NRF53_XOSC32_CAPACITANCE_X2 16  ///< Depends on the 32MHz crytal used, its capacitance is 8pF on nRF5340-DK
#endif
#endif

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

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 1) {}

#if defined(NRF5340_XXAA)
#if defined(NRF_APPLICATION)
    // Configure the 32MHz internal capacitance value (taken from crystal specs)
    const uint32_t slope_sign      = ((FICR_XOSC32MTRIM_SLOPE_Msk >> FICR_XOSC32MTRIM_SLOPE_Pos) - ((FICR_XOSC32MTRIM_SLOPE_Msk >> FICR_XOSC32MTRIM_SLOPE_Pos) >> 1));
    const int32_t  slope           = (int32_t)(((NRF_FICR_S->XOSC32MTRIM & FICR_XOSC32MTRIM_SLOPE_Msk) >> FICR_XOSC32MTRIM_SLOPE_Pos) ^ slope_sign) - (int32_t)slope_sign;
    const uint32_t offset          = (uint32_t)(NRF_FICR_S->XOSC32MTRIM & FICR_XOSC32MTRIM_OFFSET_Msk) >> FICR_XOSC32MTRIM_OFFSET_Pos;
    const uint32_t cap_value       = ((((slope + 56) * (NRF53_XOSC32_CAPACITANCE_X2 - 14)) + ((offset - 8) << 4) + 32) >> 6);
    NRF_OSCILLATORS_S->XOSC32MCAPS = (OSCILLATORS_XOSC32MCAPS_ENABLE_Enabled << OSCILLATORS_XOSC32MCAPS_ENABLE_Pos) | cap_value;
#endif
    NRF_CLOCK->HFCLKSRC = CLOCK_HFCLKSRC_SRC_HFXO << CLOCK_HFCLKSRC_SRC_Pos;
    // Enable 128MHZ core clock, only possible with application core
    NRF_CLOCK->HFCLKCTRL = CLOCK_HFCLKCTRL_HCLK_Div1 << CLOCK_HFCLKCTRL_HCLK_Pos;
#endif

    NRF_CLOCK->TASKS_HFCLKSTART = 1;
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
#if defined(NRF_APPLICATION)
    // To use LFXO, P0.0 and P0.1 must use value Peripheral in MCUSEL bitfield
    NRF_P0_S->PIN_CNF[0] = GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos;
    NRF_P0_S->PIN_CNF[1] = GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos;

    // Apply internal capacitor values as defined in 32k crystal specs
#if defined(BOARD_DOTBOT_V2)
    NRF_OSCILLATORS_S->XOSC32KI.INTCAP = OSCILLATORS_XOSC32KI_INTCAP_INTCAP_C7PF << OSCILLATORS_XOSC32KI_INTCAP_INTCAP_Pos;
#else
    NRF_OSCILLATORS_S->XOSC32KI.INTCAP = OSCILLATORS_XOSC32KI_INTCAP_INTCAP_C9PF << OSCILLATORS_XOSC32KI_INTCAP_INTCAP_Pos;
#endif

    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_LFXO << CLOCK_LFCLKSRC_SRC_Pos);
#else   // NRF_NETWORK use internal RC oscillator
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_LFRC << CLOCK_LFCLKSRC_SRC_Pos);
#endif  // defined(NRF_APPLICATION)
#else   // nrf52
    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
#endif  // defined(NRF5340_XXAA)

    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
    _clock_state.lf_enabled = true;
}
