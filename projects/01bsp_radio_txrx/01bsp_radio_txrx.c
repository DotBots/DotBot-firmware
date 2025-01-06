/**
 * @file
 * @ingroup samples_bsp
 *
 * @brief This is a short example of how to interface with the onboard Radio in the DotBot board.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 *
 * @copyright Inria, 2022-2024
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

// See conf.h

//=========================== variables =========================================

static const uint8_t packet_tx[] = {
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D,  // --------
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,  // AAAAAAAA
    0x31, 0x32, 0x33, 0x34, 0x00,                    // 1234 + null
};

static const gpio_t _dbg_pin = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };

//=========================== functions =========================================

//static void radio_callback(uint8_t *packet, uint8_t length) {
//    db_gpio_toggle(&_dbg_pin);
//    printf("(%dB): %s, RSSI: %i\n", length, (char *)packet, db_radio_rssi());
//}

static void radio_callback(uint8_t *packet, uint8_t length) {
    printf("Received packet of length %d\n", length);
    for (uint8_t i = 0; i < length; i++) {
        printf("%02x ", packet[i]);
    }
    puts("");
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
    db_timer_hf_init(0);

    //=========================== Configure Radio ===============================

    db_radio_init(&radio_callback, DB_RADIO_BLE_2MBit);
    db_radio_set_frequency(2); // 2 is one of the BLE advertising frequencies
    db_radio_rx();
    puts("Receiving on frequency 2...");

    while (1) {
        //db_radio_disable();
        //db_radio_tx((uint8_t *)packet_tx, sizeof(packet_tx) / sizeof(packet_tx[0]));
        db_timer_hf_delay_ms(0, 10);
    }
}
