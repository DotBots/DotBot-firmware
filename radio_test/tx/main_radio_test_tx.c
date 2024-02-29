/**
 * @file
 * @defgroup project_Radio    Radio TX test application
 * @ingroup projects
 * @brief This is the radio tx code for radio test

 *
 * @author Raphael Simoes <raphael.simoes@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Include BSP headers
#include "board_config.h"
#include "gpio.h"
#include "radio.h"
#include "timer.h"

static const gpio_t db_gpio_0_9 = { .port = 0, .pin = 9 };

static void _led1_blink_slow(void) {
    db_gpio_toggle(&db_led1);
}

static void _gpio_square_signal(void) {
    db_gpio_toggle(&db_gpio_0_9);
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 1000, _led1_blink_slow);
    db_timer_set_periodic_ms(1, 1, _gpio_square_signal);

    db_radio_init(NULL, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_tx_power(RADIO_TXPOWER_TXPOWER_Pos8dBm);

    uint8_t packet_counter[100] = { 0 };
    uint8_t i;

    while (1) {
        db_radio_disable();

        db_radio_tx(&packet_counter[0], 100 * sizeof(uint8_t));
        packet_counter[0]++;
        i = 1;
        while (packet_counter[i - 1] == 0 && i < 100) {
            packet_counter[i++]++;
        }

        db_timer_delay_ms(10);
    }
}
