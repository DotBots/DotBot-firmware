/**
 * @file
 * @defgroup project_Radio    Radio RX test application
 * @ingroup projects
 * @brief This is the radio rx code for radio test

 *
 * @author Raphael Simoes <raphael.simoes@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdbool.h>
#include <string.h>
// Include BSP headers
#include "uart.h"
#include "gpio.h"
#include "board_config.h"
#include "radio_ieee_802154.h"
#include "hdlc.h"
#include "timer.h"

#define DB_BUFFER_MAX_BYTES (255U)       ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)  ///< UART baudrate used by the gateway
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_UART_INDEX (1)  ///< Index of UART peripheral to use
#else
#define DB_UART_INDEX (0)  ///< Index of UART peripheral to use
#endif

#define payload_size 100

typedef struct __attribute__((packed)) {
    uint8_t payload[payload_size];
    int8_t  length;
    int8_t  rssi;
    bool    crc;
} radio_test_data_t;

static radio_test_data_t _data = { 0 };
static uint8_t           hdlc_tx_buffer[DB_BUFFER_MAX_BYTES];
static size_t            hdlc_tx_buffer_size;
int                      i;

static void _led1_blink(void) {
    db_gpio_toggle(&db_led1);
}

static void _radio_callback(uint8_t *packet, uint8_t length, bool crc) {

    for (i = 0; i < payload_size; i++) {
        _data.payload[i] = packet[i];
    }
    _data.length = length;
    _data.rssi   = db_radio_ieee_802154_rssi();
    _data.crc    = crc;

    hdlc_tx_buffer_size = db_hdlc_encode((uint8_t *)&_data, sizeof(radio_test_data_t), hdlc_tx_buffer);
    db_uart_write(DB_UART_INDEX, hdlc_tx_buffer, hdlc_tx_buffer_size);
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_timer_init();
    db_timer_set_periodic_ms(0, 2000, _led1_blink);

    db_radio_ieee_802154_init(_radio_callback);
    db_radio_ieee_802154_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_ieee_802154_rx();

    db_uart_init(DB_UART_INDEX, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, NULL);

    while (1) {}
}
