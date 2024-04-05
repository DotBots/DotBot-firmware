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

#define mode 1  // Mode focus the payload 1) Tone 2) BLE_1MBit 3) 802.15.4  Mode Always on 4)Tone

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#endif

// time in µs
#define t_start_synchro 14
#define t_adress_send   48
#define t_length_send   8
#define t_min_wait      1040
#define t_bytes         8

#define payload_size 75

static const gpio_t db_gpio_0_6            = { .port = 0, .pin = 8 };  // P0.06
static const gpio_t db_gpio_0_7            = { .port = 0, .pin = 7 };  // P0.07
static uint8_t      _payload[payload_size] = { 0 };
uint8_t             radio_tx_bytes         = 0;
int                 i                      = 0;

static void _led1_blink_slow(void) {
    db_gpio_toggle(&db_led1);
}

static void _disable_radio(void) {
    db_radio_disable();
}

static void _enable_radio(void) {
    int j;
    if (mode == 1) {
        db_radio_tx_start();
    } else {
        for (j = 0; j < payload_size; j++)
            _payload[j] = (rand() % 255);
        db_radio_disable();
        db_radio_tx(&_payload[0], payload_size * sizeof(uint8_t));
    }
}

static void _gpio_callback(void *ctx) {
    (void)ctx;

    if (i > 500 && i < 600) {
        db_timer_hf_set_oneshot_us(0, t_start_synchro + t_adress_send + t_length_send, _enable_radio);
        if (mode == 1) {
            db_timer_hf_set_oneshot_us(1, t_min_wait + t_adress_send + t_length_send + t_bytes * 2, _disable_radio);
        }
    }
    i++;
}

int main(void) {
#if !(defined(NRF5340_XXAA) && defined(NRF_APPLICATION))
    // Debug radio Blocker visualisation
    uint32_t event_tx_ready    = (uint32_t)&NRF_RADIO->EVENTS_TXREADY;
    uint32_t event_tx_disabled = (uint32_t)&NRF_RADIO->EVENTS_DISABLED;
    uint32_t task_gpiote_set   = (uint32_t)&NRF_GPIOTE->TASKS_SET[1];
    uint32_t task_gpiote_clr   = (uint32_t)&NRF_GPIOTE->TASKS_CLR[1];

    NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                            (db_gpio_0_6.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (db_gpio_0_6.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_None << GPIOTE_CONFIG_POLARITY_Pos) |
                            (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_PPI->CHEN = 1 << 0 | 1 << 1;

    NRF_PPI->CH[0].EEP = event_tx_ready;
    NRF_PPI->CH[0].TEP = task_gpiote_set;

    NRF_PPI->CH[1].EEP = event_tx_disabled;
    NRF_PPI->CH[1].TEP = task_gpiote_clr;
#endif

    // Init timer ,radio and gpio
    db_gpio_init(&db_led1, DB_GPIO_OUT);

    db_timer_init();
    db_timer_set_periodic_ms(0, 2000, _led1_blink_slow);
    db_timer_hf_init();

    if (mode == 3) {
        db_radio_init(NULL, DB_RADIO_IEEE802154_250Kbit);
    } else {
        db_radio_init(NULL, DB_RADIO_BLE_1MBit);
    }

    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_set_tx_power(RADIO_TXPOWER_TXPOWER_0dBm);

    // srand( time( NULL ) );
    //  Send RSSI
    for (i = 0; i < 100; i++) {
        db_radio_disable();
        db_radio_tx(&_payload[0], payload_size * sizeof(uint8_t));
        db_timer_delay_ms(100);
    }
    i = 0;
    db_radio_disable();

    db_gpio_init_irq(&db_gpio_0_7, DB_GPIO_IN, DB_GPIO_IRQ_EDGE_RISING, _gpio_callback, NULL);  // can receive GPIO IRq from Tx and begin to block

    while (1) {}
}
