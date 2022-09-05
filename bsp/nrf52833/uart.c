/**
 * @file uart.c
 * @addtogroup BSP
 * 
 * @brief  nRF52833-specific definition of the "uart" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

#include "gpio.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_UARTE          (NRF_UARTE0)
#define DB_UARTE_IRQ      (UARTE0_UART0_IRQn)
#define DB_UARTE_ISR      (UARTE0_UART0_IRQHandler)
#define DB_UARTE_BAUDRATE (UARTE_BAUDRATE_BAUDRATE_Baud1M)

typedef struct {
    uint8_t      byte;      ///< the byte where received byte on UART is stored
    uart_rx_cb_t callback;  ///< pointer to the callback function
} uart_vars_t;

//=========================== variables ========================================

static uart_vars_t _uart_vars;  ///< variable handling the UART context

//=========================== public ===========================================

void db_uart_init(const gpio_t *rx_pin, const gpio_t *tx_pin, uart_rx_cb_t callback) {

    // configure UART pins (RX as input, TX as output);
    nrf_port[rx_pin->port]->PIN_CNF[rx_pin->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    nrf_port[rx_pin->port]->PIN_CNF[tx_pin->pin] &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    nrf_port[rx_pin->port]->DIRSET = (1 << tx_pin->pin);

    // configure UART
    DB_UARTE->CONFIG   = 0;
    DB_UARTE->PSEL.RXD = (rx_pin->port << UARTE_PSEL_RXD_PORT_Pos) |
                         (rx_pin->pin << UARTE_PSEL_RXD_PIN_Pos) |
                         (UARTE_PSEL_RXD_CONNECT_Connected << UARTE_PSEL_RXD_CONNECT_Pos);
    DB_UARTE->PSEL.TXD = (tx_pin->port << UARTE_PSEL_TXD_PORT_Pos) |
                         (tx_pin->pin << UARTE_PSEL_TXD_PIN_Pos) |
                         (UARTE_PSEL_TXD_CONNECT_Connected << UARTE_PSEL_TXD_CONNECT_Pos);
    DB_UARTE->PSEL.RTS = 0xffffffff;  // pin disconnected
    DB_UARTE->PSEL.CTS = 0xffffffff;  // pin disconnected
    DB_UARTE->BAUDRATE = (DB_UARTE_BAUDRATE << UARTE_BAUDRATE_BAUDRATE_Pos);
    DB_UARTE->ENABLE   = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    if (callback) {
        _uart_vars.callback     = callback;
        DB_UARTE->RXD.MAXCNT    = 1;
        DB_UARTE->RXD.PTR       = (uint32_t)&_uart_vars.byte;
        DB_UARTE->INTENSET      = (UARTE_INTENSET_ENDRX_Enabled << UARTE_INTENSET_ENDRX_Pos);
        DB_UARTE->SHORTS        = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);
        DB_UARTE->TASKS_STARTRX = 1;
        NVIC_EnableIRQ(DB_UARTE_IRQ);
    }
}

void db_uart_write(uint8_t *buffer, size_t length) {
    DB_UARTE->EVENTS_ENDTX  = 0;
    DB_UARTE->TXD.PTR       = (uint32_t)buffer;
    DB_UARTE->TXD.MAXCNT    = length;
    DB_UARTE->TASKS_STARTTX = 1;
    while (DB_UARTE->EVENTS_ENDTX == 0) {}
}

//=========================== interrupts =======================================

void DB_UARTE_ISR(void) {
    // check if the interrupt was caused by a fully received package
    if (DB_UARTE->EVENTS_ENDRX) {
        DB_UARTE->EVENTS_ENDRX = 0;
        // make sure we actually received new data
        if (DB_UARTE->RXD.AMOUNT != 0) {
            // process received byte
            _uart_vars.callback(_uart_vars.byte);
        }
    }
    NVIC_ClearPendingIRQ(DB_UARTE_IRQ);
}
