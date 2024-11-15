/**
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application can be flashed on partition 0 (at 0x2000)
 *
 * @copyright Inria, 2023
 *
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <nrf.h>

#include "board_config.h"
#include "gpio.h"
#include "hdlc.h"
#include "radio.h"
#include "timer.h"
#include "uart.h"

#include "upgate.h"

//=========================== defines ==========================================

#define DB_UART_BAUDRATE (1000000U)

typedef struct {
    bool    packet_received;
    uint8_t message_buffer[UINT8_MAX];
    uint8_t uart_byte;
    bool    uart_byte_received;
    uint8_t hdlc_buffer[UINT8_MAX];
} application_vars_t;

//=========================== variables ========================================

static application_vars_t _app_vars = { 0 };

#if defined(BOARD_NRF52833DK)
static const gpio_t _sck_pin  = { .port = 0, .pin = 23 };
static const gpio_t _miso_pin = { .port = 0, .pin = 22 };
static const gpio_t _mosi_pin = { .port = 0, .pin = 21 };
static const gpio_t _cs_pin   = { .port = 0, .pin = 20 };
static const gpio_t _prog_pin = { .port = 0, .pin = 19 };
#else
static const gpio_t _sck_pin  = { .port = 1, .pin = 15 };
static const gpio_t _miso_pin = { .port = 1, .pin = 14 };
static const gpio_t _mosi_pin = { .port = 1, .pin = 13 };
static const gpio_t _cs_pin   = { .port = 1, .pin = 12 };
static const gpio_t _prog_pin = { .port = 1, .pin = 11 };
#endif

static const n25q128_conf_t _n25q128_conf = {
    .mosi = &_mosi_pin,
    .sck  = &_sck_pin,
    .miso = &_miso_pin,
    .cs   = &_cs_pin,
};

//=========================== callbacks ========================================

static void _radio_callback(uint8_t *pkt, uint8_t len) {
    memcpy(&_app_vars.message_buffer, pkt, len);
    _app_vars.packet_received = true;
}

static void _uart_callback(uint8_t byte) {
    _app_vars.uart_byte          = byte;
    _app_vars.uart_byte_received = true;
}

//=========================== private ==========================================

static void _upgate_reply(const uint8_t *message, size_t len) {
    db_radio_disable();
    db_radio_tx(message, len);

    size_t frame_len = db_hdlc_encode(message, len, _app_vars.hdlc_buffer);
    db_uart_write(0, _app_vars.hdlc_buffer, frame_len);
}

static const db_upgate_conf_t _upgate_config = {
    .reply        = _upgate_reply,
    .n25q128_conf = &_n25q128_conf,
    .prog         = &_prog_pin,
};

//================================ main ========================================

int main(void) {
    db_upgate_init(&_upgate_config);

    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);
    db_radio_rx();

    db_uart_init(0, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_callback);

    // Radio feedback LED
    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_gpio_set(&db_led1);

    // UART feedback LED
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_set(&db_led2);

    while (1) {
        __WFE();

        if (_app_vars.uart_byte_received) {
            _app_vars.uart_byte_received = false;
            db_hdlc_state_t hdlc_state   = db_hdlc_rx_byte(_app_vars.uart_byte);
            switch ((uint8_t)hdlc_state) {
                case DB_HDLC_STATE_IDLE:
                case DB_HDLC_STATE_RECEIVING:
                case DB_HDLC_STATE_ERROR:
                    break;
                case DB_HDLC_STATE_READY:
                {
                    size_t msg_len = db_hdlc_decode(_app_vars.hdlc_buffer);
                    if (msg_len) {
                        db_gpio_toggle(&db_led2);
                        db_upgate_handle_message(_app_vars.hdlc_buffer);
                    }
                } break;
                default:
                    break;
            }
        }

        if (_app_vars.packet_received) {
            _app_vars.packet_received = false;
            db_gpio_toggle(&db_led1);
            db_upgate_handle_message(_app_vars.message_buffer);
        }
    }
}
