/**
 * @file
 * @ingroup bsp_spim
 *
 * @brief  nRF52833-specific definition of the "spim" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2024-present
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <nrf.h>
#include <nrf_peripherals.h>

#include "clock.h"
#include "gpio.h"
#include "spim.h"

//=========================== defines ==========================================

typedef struct {
    NRF_SPIM_Type *p;
    IRQn_Type      irq;
} spim_conf_t;

typedef struct {
    bool running;  ///< whether bytes are being sent/received
} spim_vars_t;

//=========================== variables ========================================

static const spim_conf_t _devs[SPIM_COUNT] = {
#if defined(NRF5340_XXAA)
    {
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_SPIM0_NS,
#else
        .p = NRF_SPIM0_S,
#endif
        .irq = SERIAL0_IRQn,
    },
#if defined(NRF_APPLICATION)
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_SPIM1_NS,
#else
        .p = NRF_SPIM1_S,
#endif
        .irq = SERIAL1_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_SPIM2_NS,
#else
        .p = NRF_SPIM2_S,
#endif
        .irq = SERIAL2_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_SPIM3_NS,
#else
        .p = NRF_SPIM3_S,
#endif
        .irq = SERIAL3_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_SPIM4_NS,
#else
        .p = NRF_SPIM4_S,
#endif
        .irq = SPIM4_IRQn,
    },
#endif
#else
    {
        .p   = NRF_SPIM0,
        .irq = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
    },
    {
        .p   = NRF_SPIM1,
        .irq = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
    },
    {
        .p   = NRF_SPIM2,
        .irq = SPIM2_SPIS2_SPI2_IRQn,
    },
    {
        .p   = NRF_SPIM3,
        .irq = SPIM3_IRQn,
    },
#endif
};

static spim_vars_t _spim_vars[SPIM_COUNT] = { 0 };

//=========================== public ===========================================

void db_spim_init(spim_t spim, const db_spim_conf_t *conf) {
    _spim_vars[spim].running = false;

    db_hfclk_init();

    // configure SPIM pins
    db_gpio_init(conf->mosi, DB_GPIO_IN_PD);
    db_gpio_init(conf->sck, DB_GPIO_IN_PD);
    db_gpio_init(conf->miso, DB_GPIO_IN_PD);

    nrf_port[conf->sck->port]->PIN_CNF[conf->sck->pin] |= GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;
    nrf_port[conf->mosi->port]->PIN_CNF[conf->mosi->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[conf->miso->port]->PIN_CNF[conf->miso->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);

    _devs[spim].p->PSEL.MOSI = (conf->mosi->port << SPIM_PSEL_MOSI_PORT_Pos) |
                               (conf->mosi->pin << SPIM_PSEL_MOSI_PIN_Pos) |
                               (SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos);
    _devs[spim].p->PSEL.SCK = (conf->sck->port << SPIM_PSEL_SCK_PORT_Pos) |
                              (conf->sck->pin << SPIM_PSEL_SCK_PIN_Pos) |
                              (SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos);
    _devs[spim].p->PSEL.MISO = (conf->miso->port << SPIM_PSEL_MISO_PORT_Pos) |
                               (conf->miso->pin << SPIM_PSEL_MISO_PIN_Pos) |
                               (SPIM_PSEL_MISO_CONNECT_Connected << SPIM_PSEL_MISO_CONNECT_Pos);

    NVIC_EnableIRQ(_devs[spim].irq);
    NVIC_ClearPendingIRQ(_devs[spim].irq);
}

void db_spim_begin(spim_t spim, const gpio_t *cs, db_spim_mode_t mode, uint32_t freq) {
    _devs[spim].p->CONFIG    = mode;
    _devs[spim].p->FREQUENCY = freq;
    _devs[spim].p->ENABLE    = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
    db_gpio_clear(cs);
}

void db_spim_end(spim_t spim, const gpio_t *cs) {
    db_gpio_set(cs);
    _devs[spim].p->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
}

static void _start_transfer(spim_t spim) {
    _spim_vars[spim].running   = true;
    _devs[spim].p->INTENSET    = SPIM_INTENSET_END_Enabled << SPIM_INTENSET_END_Pos;
    _devs[spim].p->EVENTS_END  = 0;
    _devs[spim].p->TASKS_START = 1;

    while (_spim_vars[spim].running) {
        __WFE();
    }
    _devs[spim].p->INTENCLR = SPIM_INTENCLR_END_Enabled << SPIM_INTENCLR_END_Pos;
}

void db_spim_send(spim_t spim, const void *bytes, size_t len) {
    _devs[spim].p->TXD.PTR = (uint32_t)bytes;
    _devs[spim].p->RXD.PTR = (uint32_t)NULL;

    _devs[spim].p->TXD.MAXCNT = len;
    _devs[spim].p->RXD.MAXCNT = 0;
    _start_transfer(spim);
}

void db_spim_receive(spim_t spim, const void *bytes, size_t len) {
    _devs[spim].p->TXD.PTR = (uint32_t)NULL;
    _devs[spim].p->RXD.PTR = (uint32_t)bytes;

    _devs[spim].p->TXD.MAXCNT = 0;
    _devs[spim].p->RXD.MAXCNT = len;
    _start_transfer(spim);
}

//=========================== interrupt ========================================

void _spim_isr(spim_t spim) {
    if (_devs[spim].p->EVENTS_END) {
        _devs[spim].p->EVENTS_END = 0;
        _spim_vars[spim].running  = false;
    }
}

#if defined(NRF5340_XXAA)
void SERIAL0_IRQHandler(void) {
    _spim_isr(0);
}
#if defined(NRF_APPLICATION)
void SERIAL1_IRQHandler(void) {
    _spim_isr(1);
}

void SERIAL2_IRQHandler(void) {
    _spim_isr(2);
}

void SERIAL3_IRQHandler(void) {
    _spim_isr(3);
}

void SPIM4_IRQHandler(void) {
    _spim_isr(4);
}
#endif

#else

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void) {
    _spim_isr(0);
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void) {
    _spim_isr(1);
}

void SPIM2_SPIS2_SPI2_IRQHandler(void) {
    _spim_isr(2);
}

void SPIM3_IRQHandler(void) {
    _spim_isr(3);
}
#endif
