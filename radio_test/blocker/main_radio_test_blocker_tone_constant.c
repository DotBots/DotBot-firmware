/**
* @file
* @defgroup project_Radio    Radio Blocker test application
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
#include "gpio.h"
#include "radio.h"

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

int main(void) {
    // Init timer ,radio and gpio
    db_gpio_init(&db_led3, DB_GPIO_OUT);
    db_gpio_set(&db_led3);

    db_radio_init(NULL, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_tx_power(RADIO_TXPOWER_TXPOWER_0dBm);
    db_radio_tx_start();

    while (1) {}
}
