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
#include "timer.h"
#include "timer_hf.h"

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

// time in µs
#define t_start_ble  10
#define t_start_ieee 5

#define payload_size 240  // 60 for BLE and 240 for ieee
#if !(defined(NRF5340_XXAA))
static const gpio_t db_gpio_0_8 = { .port = 0, .pin = 8 };  // P0.08
#endif
static const gpio_t db_gpio_0_7            = { .port = 0, .pin = 7 };  // P0.07
static uint8_t      _payload[payload_size] = { 0 };
int                 i                      = 0;

static void _led3_blink(void) {
    db_gpio_toggle(&db_led3);
}

static void _disable_radio(void) {
    db_radio_disable();
}

static void _enable_radio(void) {
    for (i = 0; i < payload_size; i++) {
        _payload[i] = (rand() % 255);
    }
    db_radio_tx(&_payload[0], payload_size * sizeof(uint8_t));
}

static void _gpio_callback(void *ctx) {
    (void)ctx;
    db_timer_hf_set_oneshot_us(0, t_start_ble, _enable_radio);
    db_timer_hf_set_oneshot_us(1, 2000, _disable_radio);
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

    db_radio_init(NULL, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_tx_power(RADIO_TXPOWER_TXPOWER_0dBm);
    db_radio_disable();

    // srand( time( NULL ) );

    db_gpio_init_irq(&db_gpio_0_7, DB_GPIO_IN, DB_GPIO_IRQ_EDGE_RISING, _gpio_callback, NULL);  // can receive GPIO IRq from Tx and begin to block

    while (1) {}
}
