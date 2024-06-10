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
#include "radio_ieee_802154.h"
#include "timer.h"
#include "timer_hf.h"

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

#define PAYLOAD_SIZE 100

static const gpio_t db_gpio_0_8 = { .port = 0, .pin = 8 };  // P0.08
#if !(defined(NRF5340_XXAA))
static const gpio_t db_gpio_0_6 = { .port = 0, .pin = 6 };  // P0.06
#endif
static uint8_t _payload[PAYLOAD_SIZE] = { 0 };
static uint8_t i;

static void _led2_blink(void) {
    db_gpio_toggle(&db_led2);
}

static void _gpio_trigger(void) {
    db_gpio_clear(&db_gpio_0_8);
    if (RADIO_MODE == 1) {
        db_radio_disable();
        return;
    }
    db_radio_ieee_802154_disable();
}

static void _radio_tx(void) {

    db_gpio_set(&db_gpio_0_8);

    if (RADIO_MODE == 1) {
        db_timer_hf_set_oneshot_us(1, 2000, _gpio_trigger);
        db_radio_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
    } else {
        db_timer_hf_set_oneshot_us(1, 6000, _gpio_trigger);
        db_radio_ieee_802154_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
    }
    _payload[0]++;
    i = 1;
    while (_payload[i - 1] == 0 && i < PAYLOAD_SIZE) {
        _payload[i++]++;
    }
}

int main(void) {
#if !(defined(NRF5340_XXAA))
    // Debug radio tx visualisation
    uint32_t event_tx_ready   = (uint32_t)&NRF_RADIO->EVENTS_TXREADY;
    uint32_t event_tx_end     = (uint32_t)&NRF_RADIO->EVENTS_END;
    uint32_t event_tx_adress  = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    uint32_t event_tx_payload = (uint32_t)&NRF_RADIO->EVENTS_PAYLOAD;
    uint32_t task_gpiote_set  = (uint32_t)&NRF_GPIOTE->TASKS_SET[0];
    uint32_t task_gpiote_clr  = (uint32_t)&NRF_GPIOTE->TASKS_CLR[0];

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                            (db_gpio_0_6.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (db_gpio_0_6.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_None << GPIOTE_CONFIG_POLARITY_Pos) |
                            (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_PPI->CHEN = 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3;  // en CH[0] to CH[3]

    NRF_PPI->CH[0].EEP = event_tx_ready;
    NRF_PPI->CH[0].TEP = task_gpiote_set;

    NRF_PPI->CH[1].EEP = event_tx_adress;
    NRF_PPI->CH[1].TEP = task_gpiote_clr;

    NRF_PPI->CH[2].EEP = event_tx_payload;
    NRF_PPI->CH[2].TEP = task_gpiote_set;

    NRF_PPI->CH[3].EEP = event_tx_end;
    NRF_PPI->CH[3].TEP = task_gpiote_clr;
#endif

    // Init timer ,radio and gpio
    db_gpio_init(&db_gpio_0_8, DB_GPIO_OUT);
    db_gpio_init(&db_led2, DB_GPIO_OUT);

    db_timer_init();
    db_timer_set_periodic_ms(0, 500, _led2_blink);

    db_timer_hf_init();
    db_timer_hf_set_periodic_us(0, 10000, _radio_tx);

    if (RADIO_MODE == 1) {
        db_radio_init(NULL, DB_RADIO_BLE_1MBit);
        db_radio_set_frequency(TX_FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_set_tx_power(POWER);
        db_radio_disable();
    } else {
        db_radio_ieee_802154_init(NULL);
        db_radio_ieee_802154_set_frequency(TX_FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_ieee_802154_set_tx_power(POWER);
        db_radio_ieee_802154_disable();
    }

    while (1) {
    }
}
