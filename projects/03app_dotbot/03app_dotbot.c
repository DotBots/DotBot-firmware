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

//=========================== variables =========================================

uint8_t command = 0x00; // variable to store the arriving command from the radio controller

//=========================== prototypes =========================================

void radio_callback(uint8_t *packet, uint8_t length);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Configure Radio as a receiver
    db_radio_init(&radio_callback); // Set the callback function.
    db_radio_set_frequency(8);      // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();           // Start receiving packets.

    // Configure Motors
    db_motors_init();

    // Wait for radio packets to arrive/
    while (1) {

        __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions =========================================

/**
 *  @brief Callback function to process received packets
 *
 * This function gets called each time a packet is received.
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void radio_callback(uint8_t *packet, uint8_t length) {

    // Compress the arriving package into a single variable
    command = (packet[0] & 0x0F) | (packet[1] & 0x0F) << 1 | (packet[2] & 0x0F) << 2 | (packet[3] & 0x0F) << 3;

    switch (command) {
    case 0:
        db_motors_setSpeed(0, 0);       // No Buttons pressed, Stop
        break;
    case 1:
        db_motors_setSpeed(70, 70);     // Forward
        break;
    case 2:
        db_motors_setSpeed(60, -60);    // Turn Right
        break;
    case 3:
        db_motors_setSpeed(100, 70);    // Forward and Right
        break;
    case 4: 
        db_motors_setSpeed(-60, 60);    // Turn Left
        break;
    case 5:
        db_motors_setSpeed(70, 100);    // Forward and Left
        break;
    case 6:
        db_motors_setSpeed(0, 0);       // Left + Right = Stop
        break;
    case 7:
        db_motors_setSpeed(70, 70);     // Forward + Left + Right = Forward
        break;
    case 8:
        db_motors_setSpeed(-70, -70);   // Backward
        break;
    case 9:
        db_motors_setSpeed(0, 0);       // Back and Forward = Stop
        break;
    case 10:
        db_motors_setSpeed(-100, -70);  // Back + Left = Stop
        break;
    case 11:
        db_motors_setSpeed(60, -60);    // Forward + Back + Right = Right
        break;
    case 12:
        db_motors_setSpeed(-70, -100);  // Back and Left
        break;
    case 13:
        db_motors_setSpeed(-60, 60);    // Forward + Back + Left = Left
        break;
    case 14:
        db_motors_setSpeed(-70, -70);   // Back + Left + Right = Back
        break;
    case 15:
        db_motors_setSpeed(0, 0);       // MASH ALL THE BUTTONS!! ... and stop the robot.
        break;
    default:
        db_motors_setSpeed(0, 0);       // Otherwise, stop the robot
    }
}
