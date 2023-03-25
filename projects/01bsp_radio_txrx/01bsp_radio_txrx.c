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

//=========================== variables =========================================

static uint8_t packet_tx[] = {
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x00
};

#if defined(NRF5340_XXAA)
static const gpio_t _dbg_pin = { .port = 0, .pin = 28 };
#else
static const gpio_t _dbg_pin = { .port = 0, .pin = 13 };
#endif

//=========================== functions =========================================

static void radio_callback(uint8_t *packet, uint8_t length) {
    printf("packet received (%dB): %s\n", length, (char *)packet);
    db_gpio_toggle(&_dbg_pin);
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
    db_radio_set_frequency(8);  // Set the RX frquency to 2408 MHz.

    while (1) {
        db_radio_rx_disable();
        db_radio_tx(packet_tx, sizeof(packet_tx) / sizeof(packet_tx[0]));
        db_radio_rx_enable();
        db_timer_delay_ms(100);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
