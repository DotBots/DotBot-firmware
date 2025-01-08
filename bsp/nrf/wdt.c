/**
 * @file
 * @ingroup bsp_wdt
 *
 * @brief  Implementation of the Watchdog timer driver
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2024
 */

#include <stdint.h>
#include <nrf.h>

#include "wdt.h"

#if defined(NRF5340_XXAA)
#if defined(NRF_APPLICATION)
#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_WDT NRF_WDT0_NS
#else
#define NRF_WDT NRF_WDT0_S
#endif
#define WDT_IRQn       WDT0_IRQn
#define WDT_IRQHandler WDT0_IRQHandler
#elif defined(NRF_NETWORK)
#define NRF_WDT NRF_WDT_NS
#endif
#endif

typedef struct {
    wdt_timeout_cb_t callback;  ///< pointer to the callback function
    void            *ctx;
} wdt_vars_t;

static wdt_vars_t _wdt_vars;

void db_wdt_init(uint32_t timeout, wdt_timeout_cb_t cb, void *ctx) {
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos |
                       WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

    // Enable reload register 0
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;

    // Configure timeout and callback
    NRF_WDT->CRV       = (timeout * 32768) - 1;
    _wdt_vars.callback = cb;
    _wdt_vars.ctx      = ctx;

    NVIC_EnableIRQ(WDT_IRQn);
    NRF_WDT->INTENSET = WDT_INTENCLR_TIMEOUT_Enabled << WDT_INTENCLR_TIMEOUT_Pos;
}

void db_wdt_start(void) {
    NRF_WDT->TASKS_START = WDT_TASKS_START_TASKS_START_Trigger << WDT_TASKS_START_TASKS_START_Pos;
}

void db_wdt_reload(void) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload << WDT_RR_RR_Pos;
}

void WDT_IRQHandler(void) {
    if (_wdt_vars.callback) {
        _wdt_vars.callback(_wdt_vars.ctx);
    }
}
