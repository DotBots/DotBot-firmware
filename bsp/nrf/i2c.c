/**
 * @file
 * @ingroup bsp_i2c
 *
 * @brief  nRF52833-specific definition of the "i2c" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <nrf.h>
#include <nrf_peripherals.h>
#include "gpio.h"
#include "i2c.h"

//=========================== defines ==========================================

#define DB_TWIM_TX_BUF_SIZE (32U)  ///< TX max buffer size

typedef struct {
    NRF_TWIM_Type *p;
    IRQn_Type      irq;
} i2c_conf_t;

typedef struct {
    uint8_t buffer[DB_TWIM_TX_BUF_SIZE];  ///< internal buffer used to send bytes on I2C bus
    bool    running;                      ///< whether bytes are being sent on the I2C bus
} i2c_tx_vars_t;

//=========================== prototypes =======================================

void _wait_for_transfer(i2c_t i2c);

//=========================== variables ========================================

static const i2c_conf_t _devs[TWIM_COUNT] = {
#if defined(NRF5340_XXAA)
    {
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_TWIM0_NS,
#else
        .p = NRF_TWIM0_S,
#endif
        .irq = SERIAL0_IRQn,
    },
#if defined(NRF_APPLICATION)
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_TWIM1_NS,
#else
        .p = NRF_TWIM1_S,
#endif
        .irq = SERIAL1_IRQn,
    },
#endif
#else
    {
        .p   = NRF_TWIM0,
        .irq = SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
    },
    {
        .p   = NRF_TWIM1,
        .irq = SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn,
    },
#endif
};

static i2c_tx_vars_t _i2c_tx_vars[TWIM_COUNT] = { 0 };

//=========================== public ===========================================

void db_i2c_init(i2c_t i2c, const gpio_t *scl, const gpio_t *sda) {
    _i2c_tx_vars[i2c].running = false;
    // clear pending errors
    _devs[i2c].p->EVENTS_ERROR = 0;
    _devs[i2c].p->ERRORSRC     = 0;
    // ensure TWIM is disabled while configuring it
    _devs[i2c].p->ENABLE = TWIM_ENABLE_ENABLE_Disabled;

    // configure TWIM pins
    nrf_port[scl->port]->PIN_CNF[scl->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                              (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[sda->port]->PIN_CNF[sda->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                              (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);

    // configure TWIM
    _devs[i2c].p->PSEL.SCL = (scl->port << TWIM_PSEL_SCL_PORT_Pos) |
                             (scl->pin << TWIM_PSEL_SCL_PIN_Pos) |
                             (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos);
    _devs[i2c].p->PSEL.SDA = (sda->port << TWIM_PSEL_SDA_PORT_Pos) |
                             (sda->pin << TWIM_PSEL_SDA_PIN_Pos) |
                             (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos);

    // set frequency
    _devs[i2c].p->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400;

    NVIC_EnableIRQ(_devs[i2c].irq);
    NVIC_ClearPendingIRQ(_devs[i2c].irq);

    _devs[i2c].p->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_begin(i2c_t i2c) {
    _devs[i2c].p->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_end(i2c_t i2c) {
    _devs[i2c].p->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_read_regs(i2c_t i2c, uint8_t addr, uint8_t reg, void *data, size_t len) {
    _devs[i2c].p->ADDRESS       = addr;
    _devs[i2c].p->TXD.MAXCNT    = 1;
    _devs[i2c].p->TXD.PTR       = (uint32_t)&reg;
    _devs[i2c].p->RXD.PTR       = (uint32_t)data;
    _devs[i2c].p->RXD.MAXCNT    = (uint8_t)len;
    _devs[i2c].p->SHORTS        = (1 << TWIM_SHORTS_LASTTX_STARTRX_Pos) | (1 << TWIM_SHORTS_LASTRX_STOP_Pos);
    _devs[i2c].p->TASKS_STARTTX = 1;
    _wait_for_transfer(i2c);
}

void db_i2c_write_regs(i2c_t i2c, uint8_t addr, uint8_t reg, const void *data, size_t len) {
    assert(len + 1 <= DB_TWIM_TX_BUF_SIZE);
    // concatenate register address and input data in a single TX buffer
    _i2c_tx_vars[i2c].buffer[0] = reg;
    memcpy(&_i2c_tx_vars[i2c].buffer[1], data, len);

    // send the content to write to the register
    _devs[i2c].p->ADDRESS       = addr;
    _devs[i2c].p->TXD.PTR       = (uint32_t)_i2c_tx_vars[i2c].buffer;
    _devs[i2c].p->TXD.MAXCNT    = (uint8_t)len + 1;
    _devs[i2c].p->SHORTS        = (1 << TWIM_SHORTS_LASTTX_STOP_Pos);
    _devs[i2c].p->TASKS_STARTTX = 1;
    _wait_for_transfer(i2c);
}

//=========================== private ==========================================

void _wait_for_transfer(i2c_t i2c) {
    _devs[i2c].p->INTENSET    = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
    _i2c_tx_vars[i2c].running = true;
    while (_i2c_tx_vars[i2c].running) {
        __WFI();
    }
}

//=========================== interrupt ========================================

static void _twim_isr(i2c_t i2c) {
    if (_devs[i2c].p->EVENTS_STOPPED) {
        _devs[i2c].p->EVENTS_STOPPED = 0;
        _i2c_tx_vars[i2c].running    = false;
    }

    if (_devs[i2c].p->EVENTS_ERROR) {
        _devs[i2c].p->EVENTS_ERROR = 0;
        _i2c_tx_vars[i2c].running  = false;
        if (_devs[i2c].p->ERRORSRC & TWIM_ERRORSRC_ANACK_Msk) {
            _devs[i2c].p->ERRORSRC = TWIM_ERRORSRC_ANACK_Msk;
            puts("NACK on address byte");
        }
        if (_devs[i2c].p->ERRORSRC & TWIM_ERRORSRC_DNACK_Msk) {
            _devs[i2c].p->ERRORSRC = TWIM_ERRORSRC_DNACK_Msk;
            puts("NACK on data byte");
        }
    }
    _devs[i2c].p->INTENCLR = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
}

#if defined(NRF5340_XXAA)
__attribute__((weak)) void SERIAL0_IRQHandler(void) {
    _twim_isr(0);
}

__attribute__((weak)) void SERIAL1_IRQHandler(void) {
    _twim_isr(1);
}
#else
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void) {
    _twim_isr(0);
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void) {
    _twim_isr(1);
}
#endif
