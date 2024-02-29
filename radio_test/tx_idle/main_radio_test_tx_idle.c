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
#include <stdbool.h>
// Include BSP headers
#include "board_config.h"
#include "gpio.h"
#include "radio.h"
#include "timer.h"

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif
static const gpio_t db_gpio_0_9 = { .port = 0, .pin = 9 };

static void _led1_blink_slow(void) {
    db_gpio_toggle(&db_led1);
}
static void _gpio_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_clear(gpio);
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 2000, _led1_blink_slow);
    db_gpio_init_irq(&db_gpio_0_9, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _gpio_callback, (void *)&db_led2);

    db_radio_init(NULL, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_tx_power(RADIO_TXPOWER_TXPOWER_0dBm);
    db_radio_tx_start();  // Start receiving packets.

    while (1) {}
}
