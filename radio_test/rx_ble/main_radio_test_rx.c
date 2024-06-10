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
#include "radio.h"
#include "radio_ieee_802154.h"
#include "hdlc.h"
#include "timer.h"
#include "timer_hf.h"

#define DB_BUFFER_MAX_BYTES (255U)       ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)  ///< UART baudrate used by the gateway
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_UART_INDEX (1)  ///< Index of UART peripheral to use
#else
#define DB_UART_INDEX (0)  ///< Index of UART peripheral to use
#endif

#define PAYLOAD_SIZE 100
#define TIMER_HF          (NRF_TIMER4)         ///< Backend TIMER peripheral used by the timer

typedef struct __attribute__((packed)) {
    uint8_t payload[PAYLOAD_SIZE];
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

    for (i = 0; i < PAYLOAD_SIZE; i++) {
        _data.payload[i] = packet[i];
    }
    _data.length = length;
    if (RADIO_MODE == 1) {
        _data.rssi = db_radio_rssi();  //  BLE
    } else {
        _data.rssi = db_radio_ieee_802154_rssi();  //  IEEE
    }
    _data.crc = crc;

    hdlc_tx_buffer_size = db_hdlc_encode((uint8_t *)&_data, sizeof(radio_test_data_t), hdlc_tx_buffer);
    db_uart_write(DB_UART_INDEX, hdlc_tx_buffer, hdlc_tx_buffer_size);
}

int main(void) {
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_timer_init();
    db_timer_set_periodic_ms(0, 500, _led1_blink);

    if (RADIO_MODE == 1) {
        db_timer_hf_init();
        TIMER_HF->INTENSET                               = (1 << (TIMER_INTENSET_COMPARE0_Pos + 0));   
        TIMER_HF->CC[0] = 20;


        uint32_t event_tx_adress  = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
        uint32_t event_timer_end = (uint32_t)&TIMER_HF->EVENTS_COMPARE[0];
        uint32_t task_timer_restart  = (uint32_t)&TIMER_HF->TASKS_CLEAR;
        uint32_t task_RSSISTART  = (uint32_t)&NRF_RADIO->TASKS_RSSISTART;


        NRF_PPI->CHEN = 1 << 0 | 1 << 1 ;   // en CH[0] to CH[1]

        NRF_PPI->CH[0].EEP = event_tx_adress;
        NRF_PPI->CH[0].TEP = task_timer_restart;

        NRF_PPI->CH[1].EEP = event_timer_end;
        NRF_PPI->CH[1].TEP = task_RSSISTART;

        db_radio_init(_radio_callback, DB_RADIO_BLE_1MBit);
        db_radio_set_frequency(RX_FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_rx();
        NRF_RADIO->SHORTS   = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |             
                              (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |             
                              (RADIO_SHORTS_DISABLED_RSSISTOP_Enabled << RADIO_SHORTS_DISABLED_RSSISTOP_Pos)| 
                              (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    } else {
        db_radio_ieee_802154_init(_radio_callback);
        db_radio_ieee_802154_set_frequency(RX_FREQUENCY);  // Set the RX frequency to 2408 MHz.
        db_radio_ieee_802154_rx();
    }

    db_uart_init(DB_UART_INDEX, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, NULL);

    while (1) {}
}
