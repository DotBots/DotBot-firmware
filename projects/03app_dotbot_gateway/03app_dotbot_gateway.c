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
#include <stdbool.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "hdlc.h"
#include "protocol.h"
#include "radio.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_BUFFER_MAX_BYTES (255U)                           ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)                      ///< UART baudrate used by the gateway
#define DB_RADIO_QUEUE_SIZE (8U)                             ///< Size of the radio queue (must by a power of 2)
#define DB_UART_QUEUE_SIZE  ((DB_BUFFER_MAX_BYTES + 1) * 2)  ///< Size of the UART queue size (must by a power of 2)

typedef struct {
    uint8_t length;                       ///< Length of the radio packet
    uint8_t buffer[DB_BUFFER_MAX_BYTES];  ///< Buffer containing the radio packet
} gateway_radio_packet_t;

typedef struct {
    uint8_t                current;                       ///< Current position in the queue
    uint8_t                last;                          ///< Position of the last item added in the queue
    gateway_radio_packet_t packets[DB_RADIO_QUEUE_SIZE];  ///< Buffer containing the received bytes
} gateway_radio_packet_queue_t;

typedef struct {
    uint16_t current;                     ///< Current position in the queue
    uint16_t last;                        ///< Position of the last item added in the queue
    uint8_t  buffer[DB_UART_QUEUE_SIZE];  ///< Buffer containing the received bytes
} gateway_uart_queue_t;

typedef struct {
    db_hdlc_state_t              hdlc_state;                               ///< Current state of the HDLC decoding engine
    uint8_t                      hdlc_rx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Buffer where message received on UART is stored
    uint8_t                      hdlc_tx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Internal buffer used for sending serial HDLC frames
    uint32_t                     buttons;                                  ///< Buttons state (one byte per button)
    uint8_t                      radio_tx_buffer[DB_BUFFER_MAX_BYTES];     ///< Internal buffer that contains the command to send (from buttons)
    gateway_radio_packet_queue_t radio_queue;                              ///< Queue used to process received radio packets outside of interrupt
    gateway_uart_queue_t         uart_queue;                               ///< Queue used to process received UART bytes outside of interrupt
    bool                         handshake_done;                           ///< Whether startup handshake is done
} gateway_vars_t;

//=========================== variables ========================================

static gateway_vars_t _gw_vars;

//=========================== callbacks ========================================

static void uart_callback(uint8_t data) {
    if (!_gw_vars.handshake_done) {
        uint8_t version = DB_FIRMWARE_VERSION;
        db_uart_write(&version, 1);
        if (data == version) {
            _gw_vars.handshake_done = true;
        }
        return;
    }
    _gw_vars.uart_queue.buffer[_gw_vars.uart_queue.last] = data;
    _gw_vars.uart_queue.last                             = (_gw_vars.uart_queue.last + 1) & (DB_UART_QUEUE_SIZE - 1);
}

static void radio_callback(uint8_t *packet, uint8_t length) {
    if (!_gw_vars.handshake_done) {
        return;
    }
    memcpy(_gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].buffer, packet, length);
    _gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].length = length;
    _gw_vars.radio_queue.last                                      = (_gw_vars.radio_queue.last + 1) & (DB_RADIO_QUEUE_SIZE - 1);
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();

    // Configure Radio as transmitter
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);  // All RX packets received are forwarded in an HDLC frame over UART
    db_radio_set_frequency(8);                           // Set the radio frequency to 2408 MHz.
    // Initialize the gateway context
    _gw_vars.buttons             = 0x0000;
    _gw_vars.radio_queue.current = 0;
    _gw_vars.radio_queue.last    = 0;
    _gw_vars.handshake_done      = false;
    db_uart_init(&db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &uart_callback);

    db_radio_rx_enable();

    db_gpio_init(&db_btn2, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn3, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn4, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn1, DB_GPIO_IN_PU);

    while (1) {
        protocol_move_raw_command_t command;
        // Read Button 1 (P0.11)
        if (!db_gpio_read(&db_btn1)) {
            command.left_y = 100;
        } else if (!db_gpio_read(&db_btn2)) {
            command.left_y = -100;
        } else {
            command.left_y = 0;
        }

        // Read Button 2 (P0.12)
        if (!db_gpio_read(&db_btn3)) {
            command.right_y = 100;
        } else if (!db_gpio_read(&db_btn4)) {
            command.right_y = -100;
        } else {
            command.right_y = 0;
        }

        if (command.left_y != 0 || command.right_y != 0) {
            db_protocol_cmd_move_raw_to_buffer(_gw_vars.radio_tx_buffer, DB_BROADCAST_ADDRESS, DotBot, &command);
            db_radio_rx_disable();
            db_radio_tx(_gw_vars.radio_tx_buffer, sizeof(protocol_header_t) + sizeof(protocol_move_raw_command_t));
            db_radio_rx_enable();
        }

        while (_gw_vars.radio_queue.current != _gw_vars.radio_queue.last) {
            size_t frame_len = db_hdlc_encode(_gw_vars.radio_queue.packets[_gw_vars.radio_queue.current].buffer, _gw_vars.radio_queue.packets[_gw_vars.radio_queue.current].length, _gw_vars.hdlc_tx_buffer);
            db_uart_write(_gw_vars.hdlc_tx_buffer, frame_len);
            _gw_vars.radio_queue.current = (_gw_vars.radio_queue.current + 1) & (DB_RADIO_QUEUE_SIZE - 1);
        }

        while (_gw_vars.uart_queue.current != _gw_vars.uart_queue.last) {
            _gw_vars.hdlc_state = db_hdlc_rx_byte(_gw_vars.uart_queue.buffer[_gw_vars.uart_queue.current]);
            switch ((uint8_t)_gw_vars.hdlc_state) {
                case DB_HDLC_STATE_IDLE:
                case DB_HDLC_STATE_RECEIVING:
                case DB_HDLC_STATE_ERROR:
                    break;
                case DB_HDLC_STATE_READY:
                {
                    size_t msg_len = db_hdlc_decode(_gw_vars.hdlc_rx_buffer);
                    if (msg_len) {
                        _gw_vars.hdlc_state = DB_HDLC_STATE_IDLE;
                        db_radio_rx_disable();
                        db_radio_tx(_gw_vars.hdlc_rx_buffer, msg_len);
                        db_radio_rx_enable();
                    }
                } break;
                default:
                    break;
            }
            _gw_vars.uart_queue.current = (_gw_vars.uart_queue.current + 1) & (DB_UART_QUEUE_SIZE - 1);
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
