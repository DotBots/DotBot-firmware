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
#include "gpio.h"
#include "radio.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_UART_MAX_BYTES   (32U)   ///< Max bytes in UART receive buffer

typedef enum {
    UART_STATE_IDLE = 0,            ///< The UART is ready to start receiving messages
    UART_STATE_RECEIVING,           ///< The UART is receiving messages
} uart_state_t;

typedef struct {
    uint8_t buffer[DB_UART_MAX_BYTES];  ///< Buffer where message received on UART is stored
    uint8_t pos;                        ///< Current position in the UART buffer
} uart_message_t;

typedef struct {
    uart_message_t message;         ///< Structure that handles the UART message
    uart_state_t state;             ///< Internal state of the UART (idle or receiving)
    uint8_t expected_length;        ///< Expected length of message to receive
} uart_ctx_t;

//=========================== variables ========================================

static uart_ctx_t _uart_ctx;        // Variable handling the UART context
static const gpio_t _rx_pin = { .pin = 8, .port = 0};
static const gpio_t _tx_pin = { .pin = 6, .port = 0};

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
            if (_uart_ctx.message.pos == _uart_ctx.expected_length - 1 || _uart_ctx.message.pos == DB_UART_MAX_BYTES - 1) {
                db_radio_tx(_uart_ctx.message.buffer, _uart_ctx.expected_length);
                _uart_ctx.state = UART_STATE_IDLE;
            }
            _uart_ctx.message.pos++;
            break;
        default:
            break;
    }
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
    // Initialize the uart context
    _uart_ctx.expected_length = 0;
    _uart_ctx.state = UART_STATE_IDLE;
    db_uart_init(&_rx_pin, &_tx_pin, &uart_callback);

    while (1) {
       __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
