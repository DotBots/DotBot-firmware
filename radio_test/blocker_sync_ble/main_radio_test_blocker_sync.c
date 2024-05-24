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
#include "time.h"
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

#if !(defined(NRF5340_XXAA))
static const gpio_t db_gpio_0_8 = { .port = 0, .pin = 8 };  // P0.08
#endif
static const gpio_t db_gpio_0_7            = { .port = 0, .pin = 7 };  // P0.07
static uint8_t      _payload[PAYLOAD_SIZE] = { 0 };
int                 i                      = 0;

static void _led3_blink(void) {
    db_gpio_toggle(&db_led3);
}

static void _disable_radio(void) {
    if (RADIO_MODE == 2) {
        db_radio_ieee_802154_disable();
        return;
    }

    db_radio_disable();
}

static void _enable_radio(void) {
    if (RADIO_MODE == 3) {  // TONE
        db_radio_tx_start();
        return;
    }

    for (i = 0; i < PAYLOAD_SIZE; i++) {
        _payload[i] = (rand() % 255);
    }
    if (RADIO_MODE == 1) {  // BLE
        db_radio_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
    } else {  // 802.15.4
        db_radio_ieee_802154_tx(&_payload[0], PAYLOAD_SIZE * sizeof(uint8_t));
    }
}

static void _gpio_callback(void *ctx) {
    (void)ctx;
    db_timer_hf_set_oneshot_us(0, START_DELAY, _enable_radio);
    db_timer_hf_set_oneshot_us(1, END_DELAY, _disable_radio);
}

int main(void) {
#if !(defined(NRF5340_XXAA))
    // Debug radio Blocker visualisation
    uint32_t event_tx_ready    = (uint32_t)&NRF_RADIO->EVENTS_TXREADY;
    uint32_t event_tx_disabled = (uint32_t)&NRF_RADIO->EVENTS_DISABLED;
    uint32_t task_gpiote_set   = (uint32_t)&NRF_GPIOTE->TASKS_SET[1];
    uint32_t task_gpiote_clr   = (uint32_t)&NRF_GPIOTE->TASKS_CLR[1];

    NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                            (db_gpio_0_8.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (db_gpio_0_8.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_None << GPIOTE_CONFIG_POLARITY_Pos) |
                            (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_PPI->CHEN = 1 << 0 | 1 << 1;

    NRF_PPI->CH[0].EEP = event_tx_ready;
    NRF_PPI->CH[0].TEP = task_gpiote_set;

    NRF_PPI->CH[1].EEP = event_tx_disabled;
    NRF_PPI->CH[1].TEP = task_gpiote_clr;
#endif

    // Init timer ,radio and gpio
    db_gpio_init(&db_led3, DB_GPIO_OUT);

    db_timer_init();
    db_timer_set_periodic_ms(0, 500, _led3_blink);
    db_timer_hf_init();

    if ((RADIO_MODE == 1) || (RADIO_MODE == 3)) {  // BLE and TONE
        db_radio_init(NULL, DB_RADIO_BLE_1MBit);
        db_radio_set_frequency(FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_set_tx_power(POWER);

    } else {  // 802.15.4
        db_radio_ieee_802154_init(NULL);
        db_radio_ieee_802154_set_frequency(FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_ieee_802154_set_tx_power(POWER);
    }
    // srand( time( NULL ) );

    db_gpio_init_irq(&db_gpio_0_7, DB_GPIO_IN, DB_GPIO_IRQ_EDGE_RISING, _gpio_callback, NULL);  // can receive GPIO IRq from Tx and begin to block

    while (1) {}
}
