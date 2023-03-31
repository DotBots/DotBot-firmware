/**
 * @file 01bsp_radio_txrx.c
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
#include "gpio.h"
#include "radio.h"
#include "timer.h"

//=========================== defines ===========================================

#define DELAY_MS   (100)
#define RADIO_FREQ (8)

//=========================== variables =========================================

static const uint8_t packet_tx[] = {
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x00   // ABCDEFG
};

#if defined(NRF5340_XXAA)
static const gpio_t _dbg_pin = { .port = 0, .pin = 28 };
#else
static const gpio_t _dbg_pin = { .port = 0, .pin = 13 };
#endif

//=========================== functions =========================================

static void radio_callback(uint8_t *packet, uint8_t length) {
    db_gpio_toggle(&_dbg_pin);
    printf("packet received (%dB): %s\n", length, (char *)packet);
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
    db_timer_init();

    //=========================== Configure Radio ===============================

    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(RADIO_FREQ);  // Set the RX frquency to 2408 MHz.
    db_radio_rx_enable();

    while (1) {
        db_radio_rx_disable();
        db_radio_tx((uint8_t *)packet_tx, sizeof(packet_tx) / sizeof(packet_tx[0]));
        db_radio_rx_enable();
        db_timer_delay_ms(DELAY_MS);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
