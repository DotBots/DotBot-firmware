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

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_UARTE     (NRF_UARTE1_S)
#define DB_UARTE_IRQ (SERIAL1_IRQn)
#define DB_UARTE_ISR (SERIAL1_IRQHandler)
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define DB_UARTE     (NRF_UARTE0_NS)
#define DB_UARTE_IRQ (SERIAL0_IRQn)
#define DB_UARTE_ISR (SERIAL0_IRQHandler)
#else
#define DB_UARTE     (NRF_UARTE0)
#define DB_UARTE_IRQ (UARTE0_UART0_IRQn)
#define DB_UARTE_ISR (UARTE0_UART0_IRQHandler)
#endif
#define DB_UARTE_CHUNK_SIZE (64U)

typedef struct {
    uint8_t      byte;      ///< the byte where received byte on UART is stored
    uart_rx_cb_t callback;  ///< pointer to the callback function
} uart_vars_t;

//=========================== variables ========================================

static uart_vars_t _uart_vars;  ///< variable handling the UART context

//=========================== public ===========================================

void db_uart_init(const gpio_t *rx_pin, const gpio_t *tx_pin, uint32_t baudrate, uart_rx_cb_t callback) {

    // configure UART pins (RX as input, TX as output);
    db_gpio_init(rx_pin, DB_GPIO_IN_PU);
    db_gpio_init(tx_pin, DB_GPIO_OUT);

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

    // configure baudrate
    switch (baudrate) {
        case 1200:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud1200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 9600:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 14400:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud14400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 19200:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud19200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 28800:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud28800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 31250:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud31250 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 38400:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud38400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 56000:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud56000 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 57600:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud57600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 76800:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud76800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 115200:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 230400:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud230400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 250000:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud250000 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 460800:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud460800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 921600:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud921600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 1000000:
            DB_UARTE->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud1M << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        default:
            // error, return without enabling UART
            return;
    }

    DB_UARTE->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    if (callback) {
        _uart_vars.callback     = callback;
        DB_UARTE->RXD.MAXCNT    = 1;
        DB_UARTE->RXD.PTR       = (uint32_t)&_uart_vars.byte;
        DB_UARTE->INTENSET      = (UARTE_INTENSET_ENDRX_Enabled << UARTE_INTENSET_ENDRX_Pos);
        DB_UARTE->SHORTS        = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);
        DB_UARTE->TASKS_STARTRX = 1;
        NVIC_EnableIRQ(DB_UARTE_IRQ);
        NVIC_SetPriority(DB_UARTE_IRQ, 0);
        NVIC_ClearPendingIRQ(DB_UARTE_IRQ);
    }
}

void db_uart_write(uint8_t *buffer, size_t length) {
    uint8_t pos = 0;
    // Send DB_UARTE_CHUNK_SIZE (64 Bytes) maximum at a time
    while ((pos % DB_UARTE_CHUNK_SIZE) == 0 && pos < length) {
        DB_UARTE->EVENTS_ENDTX = 0;
        DB_UARTE->TXD.PTR      = (uint32_t)&buffer[pos];
        if ((pos + DB_UARTE_CHUNK_SIZE) > length) {
            DB_UARTE->TXD.MAXCNT = length - pos + 1;
        } else {
            DB_UARTE->TXD.MAXCNT = DB_UARTE_CHUNK_SIZE;
        }
        DB_UARTE->TASKS_STARTTX = 1;
        while (DB_UARTE->EVENTS_ENDTX == 0) {}
        pos += DB_UARTE_CHUNK_SIZE;
    }
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
}
