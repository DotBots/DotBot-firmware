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
#include "protocol.h"
#include "radio.h"
#include "timer.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_UART_MAX_BYTES (32U)  ///< Max bytes in UART receive buffer

typedef enum {
    UART_STATE_IDLE = 0,   ///< The UART is ready to start receiving messages
    UART_STATE_RECEIVING,  ///< The UART is receiving messages
} uart_state_t;

typedef struct {
    uint8_t buffer[DB_UART_MAX_BYTES];  ///< Buffer where message received on UART is stored
    uint8_t pos;                        ///< Current position in the UART buffer
} uart_message_t;

typedef struct {
    uart_message_t message;   ///< Structure that handles the UART message
    uart_state_t state;       ///< Internal state of the UART (idle or receiving)
    uint8_t expected_length;  ///< Expected length of message to receive
} uart_vars_t;

//=========================== variables ========================================

static uart_vars_t _uart_vars;  // Variable handling the UART context
static const gpio_t _rx_pin = { .pin = 8, .port = 0 };
static const gpio_t _tx_pin = { .pin = 6, .port = 0 };

static uint32_t buttons = 0x0000;
static uint8_t tx_buffer[32];

//=========================== prototypes =======================================

static void _init_buttons(void);

//=========================== callbacks ========================================

static void uart_callback(uint8_t data) {
    switch (_uart_vars.state) {
    case UART_STATE_IDLE:
        _uart_vars.expected_length = data;
        _uart_vars.message.pos     = 0;
        _uart_vars.state           = UART_STATE_RECEIVING;
        break;
    case UART_STATE_RECEIVING:
        _uart_vars.message.buffer[_uart_vars.message.pos] = data;
        if (_uart_vars.message.pos == _uart_vars.expected_length - 1 || _uart_vars.message.pos == DB_UART_MAX_BYTES - 1) {
            db_radio_tx(_uart_vars.message.buffer, _uart_vars.expected_length);
            _uart_vars.state = UART_STATE_IDLE;
        }
        _uart_vars.message.pos++;
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
    db_timer_init();

    // Configure Radio as transmitter
    db_radio_init(NULL);        // Set the callback function.
    db_radio_set_frequency(8);  // Set the radio frequency to 2408 MHz.
    // Initialize the uart context
    _uart_vars.expected_length = 0;
    _uart_vars.state           = UART_STATE_IDLE;
    db_uart_init(&_rx_pin, &_tx_pin, &uart_callback);

    _init_buttons();

    while (1) {
        protocol_move_raw_command_ht command;
        buttons = NRF_P0->IN;
        // Read Button 1 (P0.11)
        if (!(buttons & GPIO_IN_PIN11_Msk)) {
            command.left_y = 80;
        } else if (!(buttons & GPIO_IN_PIN24_Msk)) {
            command.left_y = -80;
        } else {
            command.left_y = 0;
        }

        // Read Button 2 (P0.12)
        if (!(buttons & GPIO_IN_PIN12_Msk)) {
            command.right_y = 80;
        } else if (!(buttons & GPIO_IN_PIN25_Msk)) {
            command.right_y = -80;
        } else {
            command.right_y = 0;
        }

        db_protocol_cmd_move_raw_to_buffer(tx_buffer, &command);
        db_radio_tx(tx_buffer, 2 + sizeof(protocol_move_raw_command_ht));
        db_timer_delay_ms(50);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== private functions ================================

static void _init_buttons(void) {

    NRF_P0->PIN_CNF[11] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Activate the Pull-up resistor

    NRF_P0->PIN_CNF[12] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Activate the Pull-up resistor

    NRF_P0->PIN_CNF[24] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Activate the Pull-up resistor

    NRF_P0->PIN_CNF[25] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Activate the Pull-up resistor
}
