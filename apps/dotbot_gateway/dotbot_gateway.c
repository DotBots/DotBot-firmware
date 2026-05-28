#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <nrf.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "frame.h"
#include "gpio.h"
#include "hdlc.h"
#include "protocol.h"
#include "radio.h"
#include "timer.h"
#include "uart.h"

/// MarilibEdge UART event type for in-band data frames. Mirrors
/// `MARI_EDGE_DATA` from `mari/firmware/mari/models.h` (= 3). The gateway
/// wraps every received bare-radio frame with this byte before HDLC-
/// encoding, so PyDotBot's MarilibEdge adapter parses it identically to
/// a real Mari gateway's emit.
#define DB_EDGE_EVENT_DATA (3)

//=========================== defines ==========================================

#define TIMER_DEV           (1)
#define DB_BUFFER_MAX_BYTES (255U)       ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)  ///< UART baudrate used by the gateway
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_UART_INDEX (1)  ///< Index of UART peripheral to use
#else
#define DB_UART_INDEX (0)  ///< Index of UART peripheral to use
#endif
#define DB_RADIO_QUEUE_SIZE (8U)                             ///< Size of the radio queue (must by a power of 2)
#define DB_UART_QUEUE_SIZE  ((DB_BUFFER_MAX_BYTES + 1) * 2)  ///< Size of the UART queue size (must by a power of 2)
#define RADIO_APP           (DotBot)                         // DotBot Radio App

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
    uint8_t                      hdlc_rx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Buffer where message received on UART is stored
    uint8_t                      hdlc_tx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Internal buffer used for sending serial HDLC frames
    uint32_t                     buttons;                                  ///< Buttons state (one byte per button)
    uint8_t                      radio_tx_buffer[DB_BUFFER_MAX_BYTES];     ///< Internal buffer that contains the command to send (from buttons)
    gateway_radio_packet_queue_t radio_queue;                              ///< Queue used to process received radio packets outside of interrupt
    gateway_uart_queue_t         uart_queue;                               ///< Queue used to process received UART bytes outside of interrupt
    bool                         led1_blink;                               ///< Whether the status LED should blink
} gateway_vars_t;

//=========================== variables ========================================

static gateway_vars_t _gw_vars;

//=========================== callbacks ========================================

static void _uart_callback(uint8_t data) {
    _gw_vars.uart_queue.buffer[_gw_vars.uart_queue.last] = data;
    _gw_vars.uart_queue.last                             = (_gw_vars.uart_queue.last + 1) & (DB_UART_QUEUE_SIZE - 1);
}

static void _radio_callback(uint8_t *packet, uint8_t length) {
    memcpy(_gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].buffer, packet, length);
    _gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].length = length;
    _gw_vars.radio_queue.last                                      = (_gw_vars.radio_queue.last + 1) & (DB_RADIO_QUEUE_SIZE - 1);
}

static void _led1_blink_fast(void) {
    if (_gw_vars.led1_blink) {
        db_gpio_toggle(&db_led1);
    }
}

static void _led2_shutdown(void) {
    db_gpio_set(&db_led2);
}

static void _led3_shutdown(void) {
    db_gpio_set(&db_led3);
}

//=========================== private ==========================================

static void _update_move_raw_command(protocol_move_raw_command_t *command) {
    // Read Button 1 (P0.11)
    if (!db_gpio_read(&db_btn1)) {
        command->left_y = 100;
    } else if (!db_gpio_read(&db_btn2)) {
        command->left_y = -100;
    } else {
        command->left_y = 0;
    }

    // Read Button 2 (P0.12)
    if (!db_gpio_read(&db_btn3)) {
        command->right_y = 100;
    } else if (!db_gpio_read(&db_btn4)) {
        command->right_y = -100;
    } else {
        command->right_y = 0;
    }
}

//=========================== main =============================================

int main(void) {
    _gw_vars.led1_blink = true;
    // Initialize user feedback LEDs
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, 50, _led1_blink_fast);
    db_timer_set_periodic_ms(TIMER_DEV, 1, 20, _led2_shutdown);
    db_timer_set_periodic_ms(TIMER_DEV, 2, 20, _led3_shutdown);
    db_gpio_init(&db_led2, DB_GPIO_OUT);  // Packet received from Radio (e.g from a DotBot)
    db_gpio_set(&db_led2);
    db_gpio_init(&db_led3, DB_GPIO_OUT);  // Packet received from UART (e.g from the computer)
    db_gpio_set(&db_led3);

    db_board_init();

    // Configure Radio in bare mode (matches the dotbot app's RF settings).
    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_network_address(DB_FRAME_ACCESS_ADDR);
    db_radio_set_frequency(DB_FRAME_DEFAULT_FREQ);
    db_radio_rx();
    // Initialize the gateway context
    _gw_vars.buttons             = 0x0000;
    _gw_vars.radio_queue.current = 0;
    _gw_vars.radio_queue.last    = 0;
    db_uart_init(DB_UART_INDEX, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_callback);

    // Initialize buttons used to broadcast move raw values to DotBots
    db_gpio_init(&db_btn2, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn3, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn4, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn1, DB_GPIO_IN_PU);

    // Initialization done, wait a bit and shutdown status LED
    db_timer_delay_s(TIMER_DEV, 1);
    db_gpio_set(&db_led1);
    _gw_vars.led1_blink = false;

    int8_t prev_left  = 0;
    int8_t prev_right = 0;

    while (1) {
        protocol_move_raw_command_t command = { 0 };
        _update_move_raw_command(&command);

        bool send_command = (command.left_y != 0) || (command.right_y != 0) || (command.left_y == 0 && command.left_y != prev_left) || (command.right_y == 0 && command.right_y != prev_right);
        if (send_command) {

            // Build the same wire format the dotbot app expects:
            // [21B mari-shaped header][1B payload-type tag][MoveRaw payload].
            size_t tx_len                      = db_frame_header_to_buffer(_gw_vars.radio_tx_buffer, DB_BROADCAST_ADDRESS);
            _gw_vars.radio_tx_buffer[tx_len++] = DB_PROTOCOL_CMD_MOVE_RAW;
            memcpy(&_gw_vars.radio_tx_buffer[tx_len], &command, sizeof(command));
            tx_len += sizeof(command);
            db_radio_disable();
            db_radio_tx(_gw_vars.radio_tx_buffer, tx_len);
            db_radio_rx();

            prev_left  = command.left_y;
            prev_right = command.right_y;

            db_timer_delay_ms(TIMER_DEV, 50);
        }

        while (_gw_vars.radio_queue.current != _gw_vars.radio_queue.last) {
            db_gpio_clear(&db_led2);
            // Wrap the raw bare-radio frame as a MarilibEdge DATA event:
            // [DB_EDGE_EVENT_DATA (1B)][frame bytes]. The host's MarilibEdge
            // adapter strips the event byte and parses the rest as a mari Frame.
            gateway_radio_packet_t *pkt = &_gw_vars.radio_queue.packets[_gw_vars.radio_queue.current];
            uint8_t                 edge_buf[1 + DB_BUFFER_MAX_BYTES];
            edge_buf[0] = DB_EDGE_EVENT_DATA;
            memcpy(&edge_buf[1], pkt->buffer, pkt->length);
            size_t frame_len = db_hdlc_encode(edge_buf, 1 + pkt->length, _gw_vars.hdlc_tx_buffer);
            db_uart_write(DB_UART_INDEX, _gw_vars.hdlc_tx_buffer, frame_len);
            _gw_vars.radio_queue.current = (_gw_vars.radio_queue.current + 1) & (DB_RADIO_QUEUE_SIZE - 1);
        }

        while (_gw_vars.uart_queue.current != _gw_vars.uart_queue.last) {
            db_gpio_clear(&db_led3);
            db_hdlc_state_t hdlc_state = db_hdlc_rx_byte(_gw_vars.uart_queue.buffer[_gw_vars.uart_queue.current]);
            switch ((uint8_t)hdlc_state) {
                case DB_HDLC_STATE_IDLE:
                case DB_HDLC_STATE_RECEIVING:
                case DB_HDLC_STATE_ERROR:
                    break;
                case DB_HDLC_STATE_READY:
                {
                    size_t msg_len = db_hdlc_decode(_gw_vars.hdlc_rx_buffer);
                    // Host frames the TX request as [DB_EDGE_EVENT_DATA (1B)][frame bytes].
                    // Drop anything that isn't a DATA event; transmit the remainder
                    // straight onto the bare radio (the wire format already matches
                    // what dotbots expect — no re-framing).
                    if (msg_len > 1 && _gw_vars.hdlc_rx_buffer[0] == DB_EDGE_EVENT_DATA) {
                        db_radio_disable();
                        db_radio_tx(&_gw_vars.hdlc_rx_buffer[1], msg_len - 1);
                        db_radio_rx();
                    }
                } break;
                default:
                    break;
            }
            _gw_vars.uart_queue.current = (_gw_vars.uart_queue.current + 1) & (DB_UART_QUEUE_SIZE - 1);
        }
    }
}
