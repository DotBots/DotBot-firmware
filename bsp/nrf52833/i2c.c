/**
 * @file i2c.c
 * @addtogroup BSP
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
#include "gpio.h"
#include "i2c.h"

//=========================== defines ==========================================

#define DB_TWIM             (NRF_TWIM1)  ///< TWI peripheral used
#define DB_TWIM_TX_BUF_SIZE (32U)        ///< TX max buffer size

typedef struct {
    uint8_t buffer[DB_TWIM_TX_BUF_SIZE];  ///< internal buffer used to send bytes on I2C bus
    bool    running;                      ///< whether bytes are being sent on the I2C bus
} i2c_tx_vars_t;

//=========================== prototypes =======================================

void _wait_for_transfer(void);

//=========================== variables ========================================

static i2c_tx_vars_t _i2c_tx_vars;

//=========================== public ===========================================

void db_i2c_init(const gpio_t *scl, const gpio_t *sda) {
    _i2c_tx_vars.running = false;
    // clear pending errors
    DB_TWIM->EVENTS_ERROR = 0;
    DB_TWIM->ERRORSRC     = 0;
    // ensure TWIM is disabled while configuring it
    DB_TWIM->ENABLE = TWIM_ENABLE_ENABLE_Disabled;

    // configure TWIM pins
    nrf_port[scl->port]->PIN_CNF[scl->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                              (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[sda->port]->PIN_CNF[sda->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                              (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);

    // configure TWIM
    DB_TWIM->PSEL.SCL = (scl->port << TWIM_PSEL_SCL_PORT_Pos) |
                        (scl->pin << TWIM_PSEL_SCL_PIN_Pos) |
                        (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos);
    DB_TWIM->PSEL.SDA = (sda->port << TWIM_PSEL_SDA_PORT_Pos) |
                        (sda->pin << TWIM_PSEL_SDA_PIN_Pos) |
                        (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos);

    // set frequency
    DB_TWIM->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400;

    NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
    NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);

    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_begin(void) {
    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_end(void) {
    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
}

void db_i2c_read_regs(uint8_t addr, uint8_t reg, void *data, size_t len) {
    DB_TWIM->ADDRESS       = addr;
    DB_TWIM->TXD.MAXCNT    = 1;
    DB_TWIM->TXD.PTR       = (uint32_t)&reg;
    DB_TWIM->RXD.PTR       = (uint32_t)data;
    DB_TWIM->RXD.MAXCNT    = (uint8_t)len;
    DB_TWIM->SHORTS        = (1 << TWIM_SHORTS_LASTTX_STARTRX_Pos) | (1 << TWIM_SHORTS_LASTRX_STOP_Pos);
    DB_TWIM->TASKS_STARTTX = 1;
    _wait_for_transfer();
}

void db_i2c_write_regs(uint8_t addr, uint8_t reg, const void *data, size_t len) {
    assert(len + 1 <= DB_TWIM_TX_BUF_SIZE);
    // concatenate register address and input data in a single TX buffer
    _i2c_tx_vars.buffer[0] = reg;
    memcpy(&_i2c_tx_vars.buffer[1], data, len);

    // send the content to write to the register
    DB_TWIM->ADDRESS       = addr;
    DB_TWIM->TXD.PTR       = (uint32_t)_i2c_tx_vars.buffer;
    DB_TWIM->TXD.MAXCNT    = (uint8_t)len + 1;
    DB_TWIM->SHORTS        = (1 << TWIM_SHORTS_LASTTX_STOP_Pos);
    DB_TWIM->TASKS_STARTTX = 1;
    _wait_for_transfer();
}

//=========================== private ==========================================

void _wait_for_transfer(void) {
    DB_TWIM->INTENSET    = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
    _i2c_tx_vars.running = true;
    while (_i2c_tx_vars.running) {
        __WFI();
    }
}

//=========================== interrupt ========================================

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void) {
    if (DB_TWIM->EVENTS_STOPPED) {
        DB_TWIM->EVENTS_STOPPED = 0;
        _i2c_tx_vars.running    = false;
    }

    if (DB_TWIM->EVENTS_ERROR) {
        DB_TWIM->EVENTS_ERROR = 0;
        _i2c_tx_vars.running  = false;
        if (DB_TWIM->ERRORSRC & TWIM_ERRORSRC_ANACK_Msk) {
            DB_TWIM->ERRORSRC = TWIM_ERRORSRC_ANACK_Msk;
            puts("NACK on address byte");
        }
        if (DB_TWIM->ERRORSRC & TWIM_ERRORSRC_DNACK_Msk) {
            DB_TWIM->ERRORSRC = TWIM_ERRORSRC_DNACK_Msk;
            puts("NACK on data byte");
        }
    }
    DB_TWIM->INTENCLR = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
}
