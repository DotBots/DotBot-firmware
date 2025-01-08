/**
 * @file
 * @ingroup bsp_qdec
 *
 * @brief  nRF5340-specific definition of the "qdec" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>
#include <nrf_peripherals.h>
#include "gpio.h"
#include "qdec.h"

//=========================== defines ==========================================

typedef struct {
    uint32_t  overflow;
    uint32_t  underflow;
    qdec_cb_t callback;
    void     *ctx;
} qdec_vars_t;

//=========================== variables ========================================

static qdec_vars_t    _qdec_vars[QDEC_COUNT] = { 0 };
static NRF_QDEC_Type *_qdec_devs[QDEC_COUNT] = {
#if defined(NRF_TRUSTZONE_NONSECURE)
    NRF_QDEC0_NS,
    NRF_QDEC1_NS
#else
    NRF_QDEC0_S,
    NRF_QDEC1_S
#endif
};

//=========================== public ===========================================

void db_qdec_init(qdec_t qdec, const qdec_conf_t *conf, qdec_cb_t callback, void *ctx) {
    assert((qdec < QDEC_COUNT));

    // Disable before configuration
    _qdec_devs[qdec]->ENABLE = (QDEC_ENABLE_ENABLE_Disabled << QDEC_ENABLE_ENABLE_Pos);

    // Setup gpios
    db_gpio_init(conf->pin_a, DB_GPIO_IN_PU);
    db_gpio_init(conf->pin_b, DB_GPIO_IN_PU);

    _qdec_devs[qdec]->PSEL.A   = (conf->pin_a->port << QDEC_PSEL_A_PORT_Pos) | (conf->pin_a->pin << QDEC_PSEL_A_PIN_Pos) | (QDEC_PSEL_A_CONNECT_Connected << QDEC_PSEL_A_CONNECT_Pos);
    _qdec_devs[qdec]->PSEL.B   = (conf->pin_b->port << QDEC_PSEL_B_PORT_Pos) | (conf->pin_b->pin << QDEC_PSEL_B_PIN_Pos) | (QDEC_PSEL_B_CONNECT_Connected << QDEC_PSEL_B_CONNECT_Pos);
    _qdec_devs[qdec]->PSEL.LED = (QDEC_PSEL_B_CONNECT_Disconnected << QDEC_PSEL_B_CONNECT_Pos);

    // Configure callback and interrupt
    _qdec_vars[qdec].callback = callback;
    _qdec_vars[qdec].ctx      = ctx;

    // The driver is only interested in the accumulator overflow event
    _qdec_devs[qdec]->INTENSET |= (QDEC_INTENSET_ACCOF_Enabled << QDEC_INTENSET_ACCOF_Pos);
    NVIC_ClearPendingIRQ(QDEC0_IRQn + qdec);
    NVIC_EnableIRQ(QDEC0_IRQn + qdec);

    // Enable debounce filter
    _qdec_devs[qdec]->DBFEN = QDEC_DBFEN_DBFEN_Enabled << QDEC_DBFEN_DBFEN_Pos;

    // Enable and start peripheral
    _qdec_devs[qdec]->ENABLE = (QDEC_ENABLE_ENABLE_Enabled << QDEC_ENABLE_ENABLE_Pos);

    // Start
    _qdec_devs[qdec]->TASKS_START = (QDEC_TASKS_START_TASKS_START_Trigger << QDEC_TASKS_START_TASKS_START_Pos);
}

int32_t db_qdec_read(qdec_t qdec) {
    return (int32_t)_qdec_devs[qdec]->ACC + (1023 * _qdec_vars[qdec].overflow) - (1024 * _qdec_vars[qdec].underflow);
}

int32_t db_qdec_read_and_clear(qdec_t qdec) {
    _qdec_devs[qdec]->TASKS_RDCLRACC = 1;
    int32_t count                    = (int32_t)_qdec_devs[qdec]->ACCREAD + (1023 * _qdec_vars[qdec].overflow) - (1024 * _qdec_vars[qdec].underflow);
    _qdec_vars[qdec].overflow        = 0;
    _qdec_vars[qdec].underflow       = 0;
    return count;
}

static void qdec_isr(qdec_t qdec) {
    _qdec_devs[qdec]->EVENTS_ACCOF = 0;
    if (_qdec_devs[qdec]->SAMPLE < 0) {
        _qdec_vars[qdec].underflow += 1;
    } else {
        _qdec_vars[qdec].overflow += 1;
    }
    _qdec_devs[qdec]->TASKS_RDCLRACC = 1;
    if (_qdec_vars[qdec].callback) {
        _qdec_vars[qdec].callback(_qdec_vars[qdec].ctx);
    }
};

void QDEC0_IRQHandler(void) {
    qdec_isr(0);
}

void QDEC1_IRQHandler(void) {
    qdec_isr(1);
}
