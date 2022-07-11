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

#define DB_UARTE            (NRF_UARTE0)
#define DB_UARTE_IRQ        (UARTE0_UART0_IRQn)
#define DB_UARTE_ISR        (UARTE0_UART0_IRQHandler)
#define DB_UARTE_BAUDRATE   (UARTE_BAUDRATE_BAUDRATE_Baud115200)

typedef struct {
    uint8_t byte;                   /**< The byte where received byte on UART is stored */
    uart_rx_cb_t callback;          /**< Pointer to the callback function */
} uart_ctx_t;

//=========================== variables ========================================

static uart_ctx_t _uart_ctx;        // Variable handling the UART context

//=========================== public ===========================================

/**
 * @brief Initialize the UART interface
 *
 * @param[in] rx_pin    pointer to RX pin
 * @param[in] tx_pin    pointer to TX pin
 * @param[in] baudrate  baudrate of the UART
 * @param[in] callback  callback function called on each received byte
 */
void db_uart_init(const gpio_t *rx_pin, const gpio_t *tx_pin, uint32_t baudrate, uart_rx_cb_t callback) {
    // Configure UART pins (RX as input, TX as output);
    NRF_P0->PIN_CNF[rx_pin->pin] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[tx_pin->pin] &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P0->DIRSET = (1 << tx_pin->pin);

    // Configure UART
    DB_UARTE->CONFIG = 0;
    DB_UARTE->PSEL.RXD = (rx_pin->port << UARTE_PSEL_RXD_PORT_Pos) |
                        (rx_pin->pin << UARTE_PSEL_RXD_PIN_Pos) |
                        (UARTE_PSEL_RXD_CONNECT_Connected << UARTE_PSEL_RXD_CONNECT_Pos);
    DB_UARTE->PSEL.TXD = (tx_pin->port << UARTE_PSEL_TXD_PORT_Pos) |
                        (tx_pin->pin << UARTE_PSEL_TXD_PIN_Pos) |
                        (UARTE_PSEL_TXD_CONNECT_Connected << UARTE_PSEL_TXD_CONNECT_Pos);
    DB_UARTE->PSEL.RTS = 0xffffffff;    // pin disconnected
    DB_UARTE->PSEL.CTS = 0xffffffff;    // pin disconnected

    switch (baudrate) {
        case 1200:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud1200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 2400:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud2400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 4800:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud4800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 9600:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 14400:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud14400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 19200:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud19200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 28800:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud28800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 38400:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud38400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 57600:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud57600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 76800:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud76800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 115200:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 230400:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud230400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 250000:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud250000 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 460800:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud460800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 921600:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud921600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 1000000:
            DB_UARTE->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud1M << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        default:
            break;
    }

    if (callback) {
        _uart_ctx.callback = callback;
        DB_UARTE->RXD.MAXCNT = 1;
        DB_UARTE->RXD.PTR = (uint32_t)&_uart_ctx.byte;
        DB_UARTE->INTENSET = (UARTE_INTENSET_ENDRX_Enabled << UARTE_INTENSET_ENDRX_Pos);
        DB_UARTE->SHORTS = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);
        DB_UARTE->TASKS_STARTRX = 1;
        NVIC_EnableIRQ(DB_UARTE_IRQ);
    }
}

/**
 * @brief Write bytes to the UART
 *
 * @param[in] buffer    pointer to the buffer to write to UART
 * @param[in] length    number of bytes of the buffer to write
 */
void db_uart_write(uint8_t *buffer, size_t length) {
    DB_UARTE->EVENTS_ENDTX = 0;
    DB_UARTE->TXD.PTR = (uint32_t)buffer;
    DB_UARTE->TXD.MAXCNT = length;
    DB_UARTE->TASKS_STARTTX = 1;
    while (DB_UARTE->EVENTS_ENDTX == 0);
}

//=========================== interrupts =======================================

void DB_UARTE_ISR(void) {
    // Check if the interrupt was caused by a fully received package
    if (DB_UARTE->EVENTS_ENDRX) {
        DB_UARTE->EVENTS_ENDRX = 0;
        // make sure we actually received new data
        if (DB_UARTE->RXD.AMOUNT != 0) {
            // Process received byte
            _uart_ctx.callback(_uart_ctx.byte);
        }
    }
    NVIC_ClearPendingIRQ(DB_UARTE_IRQ);
}
