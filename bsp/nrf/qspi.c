/**
 * @file
 * @ingroup bsp_qspi
 *
 * @brief  Definitions of the "qspi" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2024-present
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

#include "clock.h"
#include "gpio.h"
#include "qspi.h"

//=========================== defines ==========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_TRUSTZONE_NONSECURE)
#define DB_QSPI (NRF_QSPI_NS)  ///< QSPI peripheral used
#else
#define DB_QSPI (NRF_QSPI_S)  ///< QSPI peripheral used
#endif
#else
#define DB_QSPI (NRF_QSPI)  ///< QSPI peripheral
#endif
#define DB_QSPI_IRQ         (QSPI_IRQn)        ///< SPIM IRQ
#define DB_QSPI_IRQ_HANDLER (QSPI_IRQHandler)  ///< SPIM IRQ handler function

#define DB_QSPI_STATUS_QE_BIT_Pos (6)

typedef struct {
    bool running;  ///< whether bytes are being sent/received
} qspi_vars_t;

//=========================== variables ========================================

static qspi_vars_t _qspi_vars;

static void _setup_quad_mode(bool enable) {
    uint8_t status_reg = (uint8_t)(DB_QSPI->STATUS >> QSPI_STATUS_SREG_Pos);
    if (enable) {
        status_reg |= (1 << DB_QSPI_STATUS_QE_BIT_Pos);
    } else {
        status_reg &= ~(1 << DB_QSPI_STATUS_QE_BIT_Pos);
    }
    DB_QSPI->CINSTRDAT0 = (uint32_t)status_reg;

    uint8_t opcode        = 0x01;  // Write status register
    uint8_t length        = QSPI_CINSTRCONF_LENGTH_3B;
    DB_QSPI->EVENTS_READY = 0;
    _qspi_vars.running    = true;
    DB_QSPI->CINSTRCONF   = (opcode << QSPI_CINSTRCONF_OPCODE_Pos) |
                          (length << QSPI_CINSTRCONF_LENGTH_Pos) |
                          (1 << QSPI_CINSTRCONF_LIO2_Pos) |
                          (1 << QSPI_CINSTRCONF_LIO3_Pos) |
                          (QSPI_CINSTRCONF_WIPWAIT_Disable << QSPI_CINSTRCONF_WIPWAIT_Pos) |
                          (QSPI_CINSTRCONF_WREN_Enable << QSPI_CINSTRCONF_WREN_Pos);
    while (_qspi_vars.running) {
        __WFE();
    }
}

//=========================== public ===========================================

void db_qspi_init(const db_qspi_conf_t *conf) {
    _qspi_vars.running = false;
    db_hfclk_init();

    // configure SPIM pins
    db_gpio_init(conf->io0, DB_GPIO_IN);
    db_gpio_init(conf->io1, DB_GPIO_IN);
    db_gpio_init(conf->io2, DB_GPIO_IN);
    db_gpio_init(conf->io3, DB_GPIO_IN);
    db_gpio_init(conf->cs, DB_GPIO_OUT);
    db_gpio_init(conf->sck, DB_GPIO_OUT);

    nrf_port[conf->io0->port]->PIN_CNF[conf->io0->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[conf->io1->port]->PIN_CNF[conf->io0->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[conf->io2->port]->PIN_CNF[conf->io0->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[conf->io3->port]->PIN_CNF[conf->io0->pin] |= (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
    nrf_port[conf->cs->port]->PIN_CNF[conf->cs->pin] |= GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;
    nrf_port[conf->sck->port]->PIN_CNF[conf->sck->pin] |= GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;

    DB_QSPI->PSEL.IO0 = (conf->io0->port << QSPI_PSEL_IO0_PORT_Pos) |
                        (conf->io0->pin << QSPI_PSEL_IO0_PIN_Pos) |
                        (QSPI_PSEL_IO0_CONNECT_Connected << QSPI_PSEL_IO0_CONNECT_Pos);
    DB_QSPI->PSEL.IO1 = (conf->io1->port << QSPI_PSEL_IO1_PORT_Pos) |
                        (conf->io1->pin << QSPI_PSEL_IO1_PIN_Pos) |
                        (QSPI_PSEL_IO1_CONNECT_Connected << QSPI_PSEL_IO1_CONNECT_Pos);
    DB_QSPI->PSEL.IO2 = (conf->io2->port << QSPI_PSEL_IO2_PORT_Pos) |
                        (conf->io2->pin << QSPI_PSEL_IO2_PIN_Pos) |
                        (QSPI_PSEL_IO2_CONNECT_Connected << QSPI_PSEL_IO2_CONNECT_Pos);
    DB_QSPI->PSEL.IO3 = (conf->io3->port << QSPI_PSEL_IO3_PORT_Pos) |
                        (conf->io3->pin << QSPI_PSEL_IO3_PIN_Pos) |
                        (QSPI_PSEL_IO3_CONNECT_Connected << QSPI_PSEL_IO3_CONNECT_Pos);
    DB_QSPI->PSEL.CSN = (conf->cs->port << QSPI_PSEL_CSN_PORT_Pos) |
                        (conf->cs->pin << QSPI_PSEL_CSN_PIN_Pos) |
                        (QSPI_PSEL_CSN_CONNECT_Connected << QSPI_PSEL_CSN_CONNECT_Pos);
    DB_QSPI->PSEL.SCK = (conf->sck->port << QSPI_PSEL_SCK_PORT_Pos) |
                        (conf->sck->pin << QSPI_PSEL_SCK_PIN_Pos) |
                        (QSPI_PSEL_SCK_CONNECT_Connected << QSPI_PSEL_SCK_CONNECT_Pos);

    // Configure XIP offset to 0
    DB_QSPI->XIPOFFSET = 0;

    // Configure IFCONFIG0
    uint32_t rw_opcode = (QSPI_IFCONFIG0_READOC_FASTREAD << QSPI_IFCONFIG0_READOC_Pos) |
                         (QSPI_IFCONFIG0_WRITEOC_PP << QSPI_IFCONFIG0_WRITEOC_Pos);
    if (conf->enable_quad) {
        rw_opcode = (QSPI_IFCONFIG0_READOC_READ4O << QSPI_IFCONFIG0_READOC_Pos) |
                    (QSPI_IFCONFIG0_WRITEOC_PP4O << QSPI_IFCONFIG0_WRITEOC_Pos);
    }

    DB_QSPI->IFCONFIG0 = rw_opcode |
                         (QSPI_IFCONFIG0_ADDRMODE_24BIT << QSPI_IFCONFIG0_ADDRMODE_Pos) |
                         (QSPI_IFCONFIG0_DPMENABLE_Disable << QSPI_IFCONFIG0_DPMENABLE_Pos) |
                         (QSPI_IFCONFIG0_PPSIZE_256Bytes << QSPI_IFCONFIG0_PPSIZE_Pos);

    // Configure IFCONFIG1
    DB_QSPI->IFCONFIG1 = (10 << QSPI_IFCONFIG1_SCKDELAY_Pos) |
                         (QSPI_IFCONFIG1_DPMEN_Exit << QSPI_IFCONFIG1_DPMEN_Pos) |
                         (QSPI_IFCONFIG1_SPIMODE_MODE0 << QSPI_IFCONFIG1_SPIMODE_Pos) |
                         (conf->sckfreq << QSPI_IFCONFIG1_SCKFREQ_Pos);

    DB_QSPI->INTENSET = QSPI_INTENSET_READY_Enabled << QSPI_INTENSET_READY_Pos;
    NVIC_EnableIRQ(DB_QSPI_IRQ);

    // Enable QSPI peripheral
    DB_QSPI->ENABLE = QSPI_ENABLE_ENABLE_Enabled << QSPI_ENABLE_ENABLE_Pos;

    // Clear events and activate
    DB_QSPI->EVENTS_READY   = 0;
    _qspi_vars.running      = true;
    DB_QSPI->TASKS_ACTIVATE = QSPI_TASKS_ACTIVATE_TASKS_ACTIVATE_Trigger << QSPI_TASKS_ACTIVATE_TASKS_ACTIVATE_Pos;
    while (_qspi_vars.running) {
        __WFE();
    }

    _setup_quad_mode(conf->enable_quad);
}

void db_qspi_read(const uint32_t addr, void *in, size_t len) {
    DB_QSPI->READ.SRC        = addr;
    DB_QSPI->READ.DST        = (uint32_t)in;
    DB_QSPI->READ.CNT        = (uint32_t)len;
    DB_QSPI->EVENTS_READY    = 0;
    _qspi_vars.running       = true;
    DB_QSPI->TASKS_READSTART = QSPI_TASKS_READSTART_TASKS_READSTART_Trigger << QSPI_TASKS_READSTART_TASKS_READSTART_Pos;
    while (_qspi_vars.running) {
        __WFE();
    }
}

void db_qspi_program(const uint32_t addr, const void *out, size_t len) {
    DB_QSPI->WRITE.DST        = addr;
    DB_QSPI->WRITE.SRC        = (uint32_t)out;
    DB_QSPI->WRITE.CNT        = (uint32_t)len;
    DB_QSPI->EVENTS_READY     = 0;
    _qspi_vars.running        = true;
    DB_QSPI->TASKS_WRITESTART = QSPI_TASKS_WRITESTART_TASKS_WRITESTART_Trigger << QSPI_TASKS_WRITESTART_TASKS_WRITESTART_Pos;
    while (_qspi_vars.running) {
        __WFE();
    }
}

void _erase(const uint32_t addr, uint32_t len) {
    DB_QSPI->ERASE.PTR        = addr;
    DB_QSPI->ERASE.LEN        = len;
    DB_QSPI->EVENTS_READY     = 0;
    _qspi_vars.running        = true;
    DB_QSPI->TASKS_ERASESTART = QSPI_TASKS_ERASESTART_TASKS_ERASESTART_Trigger << QSPI_TASKS_ERASESTART_TASKS_ERASESTART_Pos;
    while (_qspi_vars.running) {
        __WFE();
    }

    // Wait for the status register to be ready
    while (!(DB_QSPI->STATUS & (1 << QSPI_STATUS_READY_Pos))) {
        __WFE();
    }
}

void db_qspi_block_erase(const uint32_t addr) {
    _erase(addr, QSPI_ERASE_LEN_LEN_64KB << QSPI_ERASE_LEN_LEN_Pos);
}

void db_qspi_bulk_erase(void) {
    _erase(0, QSPI_ERASE_LEN_LEN_All << QSPI_ERASE_LEN_LEN_Pos);
}

void DB_QSPI_IRQ_HANDLER(void) {
    if (DB_QSPI->EVENTS_READY) {
        DB_QSPI->EVENTS_READY = 0;
        _qspi_vars.running    = false;
    }
}
