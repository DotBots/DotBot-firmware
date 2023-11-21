/**
 * @file
 * @ingroup samples_bsp
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the onboard Radio in the DotBot board.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Include BSP packages
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "radio.h"
#include "timer_hf.h"

//=========================== defines ===========================================

#define DELAY_MS   (100)                 // Wait 100ms between each send
#define RADIO_FREQ (8)                   // Set the frequency to 2408 MHz
#define RADIO_MODE (DB_RADIO_BLE_1MBit)  // Use BLE 1Mbit/s

//=========================== variables =========================================

static const uint8_t packet_tx[] = {
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x41,  // ABCDEFGH
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x00   // ABCDEFG
};

static const gpio_t _dbg_pin = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };

//=========================== functions =========================================

static void radio_callback(uint8_t *packet, uint8_t length) {
    db_gpio_toggle(&_dbg_pin);
    printf("packet received (%dB): %s, RSSI: %i\n", length, (char *)packet, db_radio_rssi());
}

//=========================== main ==============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    //=========================== Initialize GPIO and timer =====================

    db_gpio_init(&_dbg_pin, DB_GPIO_OUT);
    db_timer_hf_init();

    //=========================== Configure Radio ===============================

    db_radio_init(&radio_callback, RADIO_MODE);
    db_radio_set_frequency(RADIO_FREQ);
    db_radio_rx();

    while (1) {
        db_radio_disable();
        db_radio_tx((uint8_t *)packet_tx, sizeof(packet_tx) / sizeof(packet_tx[0]));
        db_timer_hf_delay_ms(DELAY_MS);
    }
}
