/**
 * @file 03app_dotbot_gateway.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief  Application that can be used as a gateway to communicate by radio with several DotBots
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "gpio.h"
#include "hdlc.h"
#include "protocol.h"
#include "radio.h"
#include "timer.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_BUFFER_MAX_BYTES (32U)  ///< Max bytes in UART receive buffer

typedef struct {
    db_hdlc_state_t hdlc_state;                        ///< Current state of the HDLC decoding engine
    uint8_t         hdlc_buffer[DB_BUFFER_MAX_BYTES];  ///< Buffer where message received on UART is stored
    uint32_t        buttons;                           ///< Buttons state (one byte per button)
    uint8_t         tx_buffer[DB_BUFFER_MAX_BYTES];    ///< Internal buffer that contains the command to send (from buttons)
} gateway_vars_t;

//=========================== variables ========================================

static const gpio_t _rx_pin = { .pin = 8, .port = 0 };
static const gpio_t _tx_pin = { .pin = 6, .port = 0 };

static gateway_vars_t _gw_vars;

//=========================== prototypes =======================================

static void _init_buttons(void);

//=========================== callbacks ========================================

static void uart_callback(uint8_t data) {
    _gw_vars.hdlc_state = db_hdlc_rx_byte(data);
    switch ((uint8_t)_gw_vars.hdlc_state) {
        case DB_HDLC_STATE_IDLE:
        case DB_HDLC_STATE_RECEIVING:
        case DB_HDLC_STATE_ERROR:
            break;
        case DB_HDLC_STATE_READY:
        {
            size_t msg_len = db_hdlc_decode(_gw_vars.hdlc_buffer);
            if (msg_len) {
                db_radio_tx(_gw_vars.hdlc_buffer, msg_len);
            }
        } break;
        default:
            break;
    }
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_timer_init();

    // Configure Radio as transmitter
    db_radio_init(NULL);        // The radio callback is not used since the gateway doesn't handle paquet received
    db_radio_set_frequency(8);  // Set the radio frequency to 2408 MHz.
    // Initialize the gateway context
    _gw_vars.buttons = 0x0000;
    db_uart_init(&_rx_pin, &_tx_pin, 1000000, &uart_callback);

    _init_buttons();

    while (1) {
        protocol_move_raw_command_ht command;
        _gw_vars.buttons = NRF_P0->IN;
        // Read Button 1 (P0.11)
        if (!(_gw_vars.buttons & GPIO_IN_PIN11_Msk)) {
            command.left_y = 100;
        } else if (!(_gw_vars.buttons & GPIO_IN_PIN24_Msk)) {
            command.left_y = -100;
        } else {
            command.left_y = 0;
        }

        // Read Button 2 (P0.12)
        if (!(_gw_vars.buttons & GPIO_IN_PIN12_Msk)) {
            command.right_y = 100;
        } else if (!(_gw_vars.buttons & GPIO_IN_PIN25_Msk)) {
            command.right_y = -100;
        } else {
            command.right_y = 0;
        }

        if (command.left_y != 0 || command.right_y != 0) {
            db_protocol_cmd_move_raw_to_buffer(_gw_vars.tx_buffer, &command);
            db_radio_tx(_gw_vars.tx_buffer, 2 + sizeof(protocol_move_raw_command_ht));
        }
        db_timer_delay_ms(20);
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
