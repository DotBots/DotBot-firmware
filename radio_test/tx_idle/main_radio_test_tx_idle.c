/**
 * @file
 * @defgroup project_Radio    Radio TX idle test application
 * @ingroup projects
 * @brief This is the radio tx enable code for radio test

 *
 * @author Raphael Simoes <raphael.simoes@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Include BSP headers
#include "board_config.h"
#include "radio.h"
#include "timer.h"

static void _led1_blink_slow(void) {
    db_gpio_toggle(&db_led1);
}

static void _radio_callback(uint8_t *packet, uint8_t length) {
    printf("packet received (%dB): %s, RSSI: %i\n", length, (char *)packet, db_radio_rssi());
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 2000, _led1_blink_slow);

    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_power(RADIO_TXPOWER_TXPOWER_0dBm);
    db_radio_tx_start();  // Start receiving packets.

    while (1) {
    }
}
