/**
 * @file
 * @ingroup bsp_saadc
 *
 * @brief  Implementation of the "saadc" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <nrf.h>
#include <nrf_peripherals.h>
#include "saadc.h"

//=========================== defines ==========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_SAADC NRF_SAADC_NS
#else
#define NRF_SAADC NRF_SAADC_S
#endif
#endif

typedef struct {
    bool    initialized;  ///< Checks whether SAADC is initialized
    int16_t result;       ///< Result of the last conversion
} saadc_vars_t;

//=========================== variables ========================================

static saadc_vars_t _saadc_vars = {
    .initialized = false,
};

static inline void _enable(void) {
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
}

static inline void _disable(void) {
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos;
}

//=========================== public ===========================================

void db_saadc_init(db_saadc_resolution_t resolution) {
    if (_saadc_vars.initialized) {
        return;
    }
    _enable();
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->RESULT.PTR    = (uint32_t)&_saadc_vars.result;

    NRF_SAADC->RESOLUTION   = resolution << SAADC_RESOLUTION_VAL_Pos;
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                              (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                              (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                              (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos);
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos;
    NRF_SAADC->OVERSAMPLE  = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;

    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0)
        ;
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos))
        ;

    _saadc_vars.initialized = true;
    _disable();
}

void db_saadc_read(db_saadc_input_t input, uint16_t *value) {
    assert(_saadc_vars.initialized);

    _enable();
    NRF_SAADC->CH[0].PSELP = input;

    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_START    = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0) {}

    NRF_SAADC->EVENTS_END   = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0) {}

    *value = _saadc_vars.result < 0 ? 0 : (uint16_t)_saadc_vars.result;

    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP     = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0) {}
    _disable();
}
