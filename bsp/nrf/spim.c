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

#include <assert.h>
#include <nrf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "spim.h"

//=========================== defines ==========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_APPLICATION)
#define DB_SPIM (NRF_SPIM0_S)  ///< SPIM peripheral used
#elif defined(NRF_NETWORK)
#define DB_SPIM (NRF_SPIM0_NS)  ///< SPIM peripheral used
#endif
#define DB_SPIM_IRQ_HANDLER (SERIAL0_IRQHandler)  ///< SPIM IRQ handler function
#define DB_SPIM_IRQ         (SERIAL0_IRQn)        ///< SPIM IRQ
#else
#define DB_SPIM             (NRF_SPIM0)                                     ///< SPIM peripheral used
#define DB_SPIM_IRQ_HANDLER (SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler)  ///< SPIM IRQ handler function
#define DB_SPIM_IRQ         (SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn)        ///< SPIM IRQ
#endif
#define DB_SPIM_BUFFER_SIZE (256U)  ///< TX max buffer size

typedef struct {
    uint8_t out_buffer[DB_SPIM_BUFFER_SIZE];  ///< internal buffer used to send bytes on SPI bus
    bool    running;                          ///< whether bytes are being sent/received
} spim_vars_t;

//=========================== variables ========================================

static spim_vars_t _spim_vars;

//=========================== public ===========================================

void db_spim_init(const db_spim_conf_t *conf) {
    _spim_vars.running = false;

    // configure SPIM pins
    db_gpio_init(conf->mosi, DB_GPIO_OUT);
    db_gpio_init(conf->sck, DB_GPIO_OUT);
    db_gpio_init(conf->miso, DB_GPIO_IN_PD);

    DB_SPIM->PSEL.MOSI = (conf->mosi->port << SPIM_PSEL_MOSI_PORT_Pos) |
                         (conf->mosi->pin << SPIM_PSEL_MOSI_PIN_Pos) |
                         (SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos);
    DB_SPIM->PSEL.SCK = (conf->sck->port << SPIM_PSEL_SCK_PORT_Pos) |
                        (conf->sck->pin << SPIM_PSEL_SCK_PIN_Pos) |
                        (SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos);
    DB_SPIM->PSEL.MISO = (conf->miso->port << SPIM_PSEL_MISO_PORT_Pos) |
                         (conf->miso->pin << SPIM_PSEL_MISO_PIN_Pos) |
                         (SPIM_PSEL_MISO_CONNECT_Connected << SPIM_PSEL_MISO_CONNECT_Pos);

    NVIC_EnableIRQ(DB_SPIM_IRQ);
    NVIC_ClearPendingIRQ(DB_SPIM_IRQ);
}

void db_spim_begin(const gpio_t *cs, db_spim_mode_t mode, uint32_t freq) {
    DB_SPIM->CONFIG    = mode;
    DB_SPIM->FREQUENCY = freq;
    DB_SPIM->ENABLE    = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
    db_gpio_clear(cs);
}

void db_spim_end(const gpio_t *cs) {
    db_gpio_set(cs);
    DB_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
}

void db_spim_transfer(const void *out, void *in, size_t len) {
    _spim_vars.running = true;
    DB_SPIM->INTENSET  = SPIM_INTENSET_END_Enabled << SPIM_INTENSET_END_Pos;
    if (out) {
        assert(len <= DB_SPIM_BUFFER_SIZE);
        memcpy(_spim_vars.out_buffer, out, len);
    }
    DB_SPIM->TXD.PTR = (uint32_t)_spim_vars.out_buffer;
    DB_SPIM->RXD.PTR = (uint32_t)in;

    DB_SPIM->TXD.MAXCNT = (out) ? len : 0;
    DB_SPIM->RXD.MAXCNT = (in) ? len : 0;

    DB_SPIM->EVENTS_END  = 0;
    DB_SPIM->TASKS_START = 1;

    while (_spim_vars.running) {
        __WFE();
    }
    DB_SPIM->INTENCLR = SPIM_INTENCLR_END_Enabled << SPIM_INTENCLR_END_Pos;
}

//=========================== interrupt ========================================

void DB_SPIM_IRQ_HANDLER(void) {
    if (DB_SPIM->EVENTS_END) {
        DB_SPIM->EVENTS_END = 0;
        _spim_vars.running  = false;
    }
}
