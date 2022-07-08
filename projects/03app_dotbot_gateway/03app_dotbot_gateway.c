/**
 * @file 03app_dotbot_gateway.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief  Application that can be used as a gateway to communicate by radio with several DotBots
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "radio.h"

//=========================== defines ==========================================

#define DB_UARTE_RX_PIN     (8)
#define DB_UARTE_RX_PORT    (0)
#define DB_UARTE_TX_PIN     (6)
#define DB_UARTE_TX_PORT    (0)

#define DB_UARTE            (NRF_UARTE0)
#define DB_UARTE_IRQ        (UARTE0_UART0_IRQn)
#define DB_UARTE_ISR        (UARTE0_UART0_IRQHandler)
#define DB_UARTE_BAUDRATE   (UARTE_BAUDRATE_BAUDRATE_Baud115200)

#define UART_MAX_BYTES 32

typedef void(*uart_rx_cb_t)(uint8_t data);

typedef enum {
    UART_STATE_IDLE = 0,            /**< The UART is ready to start receiving messages */
    UART_STATE_RECEIVING,           /**< The UART is receiving messages */
} uart_state_t;

typedef struct {
    uint8_t buffer[UART_MAX_BYTES]; /**< Buffer where message received on UART is stored */
    uint8_t pos;                    /**< Current position in the UART buffer */
} uart_message_t;

typedef struct {
    uint8_t byte;                   /**< The byte where received byte on UART is stored */
    uart_rx_cb_t callback;          /**< Pointer to the callback function */
    uart_message_t message;         /**< Structure that handles the UART message */
    uart_state_t state;             /**< Internal state of the UART (idle or receiving) */
    uint8_t expected_length;        /**< Expected length of message to receive */
} uart_ctx_t;

//=========================== variables ========================================

static uart_ctx_t _uart_ctx;        // Variable handling the UART context

//=========================== callbacks ========================================

static void uart_callback(uint8_t data) {
    switch (_uart_ctx.state) {
        case UART_STATE_IDLE:
            _uart_ctx.expected_length = data;
            _uart_ctx.message.pos = 0;
            _uart_ctx.state = UART_STATE_RECEIVING;
            break;
        case UART_STATE_RECEIVING:
            _uart_ctx.message.buffer[_uart_ctx.message.pos] = data;
            if (_uart_ctx.message.pos == _uart_ctx.expected_length - 1 || _uart_ctx.message.pos == UART_MAX_BYTES - 1) {
                db_radio_tx(_uart_ctx.message.buffer, _uart_ctx.expected_length);
                _uart_ctx.state = UART_STATE_IDLE;
            }
            _uart_ctx.message.pos++;
            break;
        default:
            break;
    }
}

//=========================== public ===========================================

static void db_uart_init(uart_rx_cb_t rx_callback) {
    // Initialize the uart context
    _uart_ctx.expected_length = 0;
    _uart_ctx.state = UART_STATE_IDLE;
    _uart_ctx.callback = rx_callback;

    // Configure UART pins (RX as input, TX as output);
    NRF_P0->PIN_CNF[DB_UARTE_RX_PIN] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[DB_UARTE_RX_PIN] &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P0->DIRSET = (1 << DB_UARTE_TX_PIN);

    // Configure UART
    DB_UARTE->CONFIG = 0;
    DB_UARTE->PSEL.RXD = (DB_UARTE_RX_PORT << UARTE_PSEL_RXD_PORT_Pos) |
                        (DB_UARTE_RX_PIN << UARTE_PSEL_RXD_PIN_Pos) |
                        (UARTE_PSEL_RXD_CONNECT_Connected << UARTE_PSEL_RXD_CONNECT_Pos);
    DB_UARTE->PSEL.TXD = (DB_UARTE_TX_PORT << UARTE_PSEL_TXD_PORT_Pos) |
                        (DB_UARTE_TX_PIN << UARTE_PSEL_TXD_PIN_Pos) |
                        (UARTE_PSEL_TXD_CONNECT_Connected << UARTE_PSEL_TXD_CONNECT_Pos);
    DB_UARTE->PSEL.RTS = 0xffffffff;    // pin disconnected
    DB_UARTE->PSEL.CTS = 0xffffffff;    // pin disconnected
    DB_UARTE->BAUDRATE = (DB_UARTE_BAUDRATE << UARTE_BAUDRATE_BAUDRATE_Pos);
    DB_UARTE->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
    DB_UARTE->RXD.MAXCNT = 1;
    DB_UARTE->RXD.PTR = (uint32_t)&_uart_ctx.byte;
    DB_UARTE->INTENSET = (UARTE_INTENSET_ENDRX_Enabled << UARTE_INTENSET_ENDRX_Pos);
    DB_UARTE->SHORTS = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);
    DB_UARTE->TASKS_STARTRX = 1;
    NVIC_EnableIRQ(DB_UARTE_IRQ);
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

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    puts("DotBot gateway application");
    db_board_init();

    NRF_P0->DIRSET = 1 << 13;
    NRF_P0->OUTCLR = 1 << 13;
    // Configure Radio as transmitter
    db_radio_init(NULL); // Set the callback function.
    db_radio_set_frequency(8);      // Set the radio frequency to 2408 MHz.
    db_uart_init(&uart_callback);

    while (1) {
       __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
