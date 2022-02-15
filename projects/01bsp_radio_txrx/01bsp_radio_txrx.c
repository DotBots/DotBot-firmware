/**
 * @file 01bsp_radio_txrx.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the onboard Radio in the DotBot board.
 *
 * This program is the RX part.
 * if the corresponding compatible TX program is loaded into a nearby nRF52840-DK,
 * pressing the buttons in the DK should toggle ON and OFF the P0.31 pin on the DotBot (P0.31).
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Inlcude BSP packages
#include <radio.h>

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== prototypes =========================================

void radio_callback(uint8_t *packet, uint8_t length); 

//=========================== main =========================================

/**
*  @brief The program starts executing here.
*/
int main(void)
{
  //=========================== Configure GPIO =========================================

  // Set the test pin (P0.31) as an output
  NRF_P0->PIN_CNF[31] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.

  //=========================== Configure Radio =========================================

  gw_radio_init(&radio_callback);   // Set the callback function.
  gw_radio_set_frequency(8);        // Set the RX frquency to 2408 MHz.
  gw_radio_rx_enable();             // Start receiving packets.

while (1) {
  
  __WFE();                          // Enter a low power state while waiting.                  
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

  // Check the arriving packet for any pressed button.
  if (packet[0] == 0x01 || packet[1] == 0x01 || packet[2] == 0x01 || packet[3] == 0x01) {

  NRF_P0->OUTSET = 1 << GPIO_OUTSET_PIN31_Pos;  // if any button is pressed, set the pin HIGH.
  }

  else {

  NRF_P0->OUTCLR = 1 << GPIO_OUTCLR_PIN31_Pos;  // No buttons pressed, set the pin LOW.
  }
}
