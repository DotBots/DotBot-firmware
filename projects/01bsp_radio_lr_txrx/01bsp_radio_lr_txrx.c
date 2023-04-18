/**
 * @file 01bsp_radio_lr_txrx.c
 * @author Trifun savic <trifun.savic@inria.fr>
 * @brief This is a short example of how to interface with the Long Range BLE radio.
 *
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Include BSP packages
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "radio.h"
#include "timer_hf.h"

//=========================== defines =========================================

#define DELAY_MS   (100)                     // Wait 100ms between each send
#define RADIO_FREQ (8)                       // Set the frequency to 2408 MHz
#define RADIO_MODE (DB_RADIO_BLE_LR125Kbit)  // Use BLE Long Range 125Kbit/s

//=========================== variables =========================================

static const uint8_t packet_tx[] = {
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x00  // ABCDEFG
};

static const gpio_t _dbg_pin = { .port = DB_DEBUG1_PORT, .pin = DB_DEBUG1_PIN };

//=========================== prototypes =========================================

static void radio_callback(uint8_t *packet, uint8_t length) {
    db_gpio_toggle(&_dbg_pin);
    printf("packet received (%dB): %s\n", length, (char *)packet);
}

//=========================== main ===============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    //=========================== Initialize GPIO and timer ======================

    db_gpio_init(&_dbg_pin, DB_GPIO_OUT);
    db_timer_hf_init();

    //=========================== Configure Radio ================================s

    db_radio_init(&radio_callback, RADIO_MODE);
    db_radio_set_frequency(RADIO_FREQ);

    while (1) {
        db_radio_rx_disable();
        db_radio_tx((uint8_t *)packet_tx, sizeof(packet_tx) / sizeof(packet_tx[0]));
        db_radio_rx_enable();
        db_timer_hf_delay_ms(DELAY_MS);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
