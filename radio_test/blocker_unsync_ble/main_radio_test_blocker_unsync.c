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
#include "radio_ieee_802154.h"
#include "timer_hf.h"

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif
#define PAYLOAD_SIZE 125
static uint8_t _payload[PAYLOAD_SIZE] = { 0 };
static uint8_t i                      = 0;

int main(void) {
    // Init timer ,radio and gpio
    db_gpio_init(&db_led3, DB_GPIO_OUT);
    db_gpio_clear(&db_led3);
    db_timer_hf_init();

    if ((RADIO_MODE == 1) || (RADIO_MODE == 3)) {
        db_radio_init(NULL, DB_RADIO_BLE_1MBit);
        db_radio_set_frequency(FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_set_tx_power(POWER);

        if (RADIO_MODE == 1) {  // BLE
            while (1) {
                _payload[0] = i;
                db_radio_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
                db_timer_hf_delay_us(2250);
                db_radio_disable();
                db_timer_hf_delay_us(10000 - 2250);
            }

        } else {  // TONE
            db_radio_tx_start();
        }

    } else {  // 802.15.4
        db_radio_ieee_802154_init(NULL);
        db_radio_ieee_802154_set_frequency(FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_ieee_802154_set_tx_power(POWER);

        while (1) {
            _payload[0] = i;
            db_radio_ieee_802154_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
            db_timer_hf_delay_us(4500);
            db_radio_ieee_802154_disable();
            db_timer_hf_delay_us(10000 - 4500);
        }
    }
}
