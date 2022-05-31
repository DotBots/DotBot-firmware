/**
 * @file 03app_dotbot_remote_control.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an application for controlling a single dotbot remotely using radio.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "motors.h"
#include "radio.h"

//=========================== defines =========================================

#define BUSY_WAIT(wait_ms) for (int i = 0; i < 3000 * wait_ms; i++) {}      ///< The 3000 magic number approximates to about 1ms.

//=========================== variables =========================================

//=========================== main =========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {

}

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    db_board_init();
    db_motors_init();
    db_radio_init(&radio_callback);
    db_radio_set_frequency(8);      // Set the RX frquency to 2408 MHz.
    db_radio_rx_enable();           // Start receiving packets.

    while (1) {
        __WFE(); // Enter a low power state while waiting.
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
