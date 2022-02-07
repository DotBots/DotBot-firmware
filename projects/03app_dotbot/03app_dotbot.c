/**
 * @file 03app_dotbot.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is the main DotBot app.
 * 
 * Load this program on your board. Now the DotBot can be remote controlled 
 * from a nearby nRF52840-DK. THe buttons of the DK serving as Forward, 
 * Right, Left and Back buttons.
 * 
 * 
 * @copyright Inria, 2022
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
// Inlcude BSP packages
#include <radio.h>
#include <motors.h>
#include <board.h>

//=========================== defines =========================================

// Define a blocking wait function.
#define WAIT_A_BIT(PAUSE) for (int i = 0; i < 3000 * PAUSE; i++) {;}    // The 3000 magic number, approximates to about 1ms per 1 unit of PAUSE.

#define NUMBER_OF_BYTES_IN_PACKET 32

//=========================== variables =========================================

static uint8_t packet[NUMBER_OF_BYTES_IN_PACKET]; // variable that stores the radio packet that arrives.

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Configure Radio as a receiver
    db_radio_rx_init(7, 0, packet);

    // Configure Motors
    db_motors_init();

    while (1) {
        // Check if a new radio packet arrived.
        if (packet[4] != 0x00) {

            // Disable the Radio while processing the data.
            db_radio_rx_disable();

            // Check each byte of the arriving packet and turn on the corresponding led

            // Check Forward button (1) only
            if (packet[0] == 0x01 && packet[1] == 0x00 && packet[2] == 0x00 && packet[3] == 0x00) {
                db_motors_setSpeed(70, 70);
            }

            // Check Right button (2) only
            if (packet[0] == 0x00 && packet[1] == 0x01 && packet[2] == 0x00 && packet[3] == 0x00) {
                db_motors_setSpeed(60, -60);
            }

            // Check Right button (3) only
            if (packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x01 && packet[3] == 0x00) {
                db_motors_setSpeed(-60, 60);
            }

            // Check Backward button (4) only
            if (packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x00 && packet[3] == 0x01) {
                db_motors_setSpeed(-70, -70);
            }

            // Check Stop condition (No buttons pressed)
            if (packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x00 && packet[3] == 0x00) {
                db_motors_setSpeed(0, 0);
            }

            // Check Forward-Right diagonal buttons (1 & 2)
            if (packet[0] == 0x01 && packet[1] == 0x01 && packet[2] == 0x00 && packet[3] == 0x00) {
                db_motors_setSpeed(100, 70);
            }

            // Check Forward-Left diagonal buttons (1 & 3)
            if (packet[0] == 0x01 && packet[1] == 0x00 && packet[2] == 0x01 && packet[3] == 0x00) {
                db_motors_setSpeed(70, 100);
            }

            // Check Back-Right diagonal buttons (4 & 2)
            if (packet[0] == 0x00 && packet[1] == 0x01 && packet[2] == 0x00 && packet[3] == 0x01) {
                db_motors_setSpeed(-100, -70);
            }

            // Check Back-Left diagonal buttons (4 & 3)
            if (packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x01 && packet[3] == 0x01) {
                db_motors_setSpeed(-70, -100);
            }

            // clear packet before next one arrives
            for (int i = 0; i < 32; i++) {
                packet[i] = 0x00;
            }

            // Renable the radio to receive further packages.
            db_radio_rx_enable();
        }
        WAIT_A_BIT(10);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
