/**
 * @file
 * @defgroup project_Radio    Radio application
 * @ingroup projects
 * @brief This is the radio rx code for radio test

 *
 * @author Raphael Simoes <raphael.simoes@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdlib.h>
#include <string.h>
// Include BSP headers
#include "uart.h"
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "protocol.h"
#include "radio.h"
#include "timer.h"

uint8_t write_crc[3] = "CRC";
uint8_t uart_end     = '\x00';

#define DB_BUFFER_MAX_BYTES (255U)       ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)  ///< UART baudrate used by the gateway
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_UART_INDEX (1)  ///< Index of UART peripheral to use
#else
#define DB_UART_INDEX (0)  ///< Index of UART peripheral to use
#endif

static void _led1_blink_fast(void) {
    db_gpio_toggle(&db_led1);
}

static void _radio_callback_crc(uint8_t *packet, uint8_t length) {
    memcpy(packet + length, &write_crc, 3);
    db_uart_write(DB_UART_INDEX, packet, length + 3);
}

static void _radio_callback(uint8_t *packet, uint8_t length) {
    uint8_t RSSI = db_radio_rssi();
    memcpy(packet + length, &RSSI, 1);
    memcpy(packet + length + 1, &uart_end, 1);
    db_uart_write(DB_UART_INDEX, packet, length + 1);
}

static void _uart_callback(uint8_t data) {
    db_uart_write(DB_UART_INDEX, &data, 1);
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 100, _led1_blink_fast);

    db_protocol_init();
    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_crc_callback(&_radio_callback_crc);
    db_radio_rx();  // Start receiving packets.

    db_uart_init(DB_UART_INDEX, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_callback);

    while (1) {
    }
}
/*************************** End of file ****************************/
