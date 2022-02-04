/**
 * @file 00std_radio_rx.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to implement radio control of the DotBot board from an external nRF52840-DK.
 * 
 * This program is the RX part.
 * 
 * If another nRF52840-DK loaded with the 00std_radio_rx project is transmiting:
 * Pressing any of the 4 buttons on the Transmitting DK, will toggle ON and OFF the corresponding LED
 * in the receiving DK.
 * 
 * 
 * @copyright Inria, 2022
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== defines =========================================

// Useful Macro functions
#define WAIT_MS(MS) for (int i = 0; i < 3000 * MS; i++) {__NOP();}   // Define a simple blocking wait milisecond function.

#define NUMBER_OF_BYTES_IN_PACKET 32

//=========================== variables =========================================

static uint8_t packet[NUMBER_OF_BYTES_IN_PACKET]; // variable that stores the radio packet that arrives.

//=========================== main =========================================

/**
*  @brief The program starts executing here.
*/
int main(void)
{

  // Turn ON the DotBot board regulator
  NRF_P0->DIRSET = 1 << 20;
  NRF_P0->OUTSET = 1 << 20;

  // Configure test GPIO (P0.31) as output
  NRF_P0->DIRSET = 1 << 31;

  //=========================== Configure Radio =========================================

  NRF_RADIO->FREQUENCY = 7UL;                                           // frequency bin 7, 2407MHz
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos); // Use Nordic proprietary 1Mbit/s protocol

  // address configuration (just random numbers I chose)
  NRF_RADIO->PREFIX0 = (0xF3UL << RADIO_PREFIX0_AP3_Pos) | // prefix byte of address 3
                       (0xF2UL << RADIO_PREFIX0_AP2_Pos) | // prefix byte of address 2
                       (0xF1UL << RADIO_PREFIX0_AP1_Pos) | // prefix byte of address 1
                       (0xF0UL << RADIO_PREFIX0_AP0_Pos);  // prefix byte of address 0

  NRF_RADIO->PREFIX1 = (0xF7UL << RADIO_PREFIX1_AP7_Pos) | // prefix byte of address 7
                       (0xF6UL << RADIO_PREFIX1_AP6_Pos) | // prefix byte of address 6
                       (0xF5UL << RADIO_PREFIX1_AP5_Pos) | // prefix byte of address 5
                       (0xF4UL << RADIO_PREFIX1_AP4_Pos);  // prefix byte of address 4

  NRF_RADIO->BASE0 = 0x14071997UL; // base address for prefix 0

  NRF_RADIO->BASE1 = 0x16081931UL; // base address for prefix 1-7

  NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos); // receive from address 0

  // packet configuration
  NRF_RADIO->PCNF0 = 0UL; //not really interested in these

  NRF_RADIO->PCNF1 = (32UL << RADIO_PCNF1_MAXLEN_Pos) |
                     (32UL << RADIO_PCNF1_STATLEN_Pos) | // since the LENGHT field is not set, this specifies the lenght of the payload
                     (4UL << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                     (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);

  // shortcuts
  // - READY and START
  // - END and START (Radio must be always listening for the packet)
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos); // |
                                                                                          //(RADIO_SHORTS_END_START_Enabled   << RADIO_SHORTS_END_START_Pos);

  // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Checksum uses 2 bytes, and is enabled.
  NRF_RADIO->CRCINIT = 0xFFFFUL;                                      // initial value.
  NRF_RADIO->CRCPOLY = 0x11021UL;                                     // CRC poly: x^16 + x^12^x^5 + 1.

  // pointer to packet payload
  NRF_RADIO->PACKETPTR = (uint32_t)&packet;

  // // Configure the external High-frequency Clock. (Needed for correct operation)
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;           // Clear the flag
  NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;           // Start the clock
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {;}  // Wait for the clock to actually start.

  // Enable the Radio
  NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;

  while (1)
  {
    if (packet[4] != 0x00)
    {

      // Disable the Radio while processing the data.
      NRF_RADIO->TASKS_DISABLE = RADIO_TASKS_DISABLE_TASKS_DISABLE_Trigger; 

      // Check each byte of the arriving packet and turn on the corresponding led
      if (packet[0] == 0x01)  { NRF_P0->OUTSET = 1 << GPIO_OUTSET_PIN31_Pos; }
      else                    { NRF_P0->OUTCLR = 1 << GPIO_OUTCLR_PIN31_Pos; }

      for (int i = 0; i < 32; i++)
      { // clear packet before next one arrives
        packet[i] = 0x00;
      }

      // Renable the radio to receive further packages.
      NRF_RADIO->TASKS_RXEN = 1UL;
    }
    WAIT_MS(10);
  }

  // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
  __NOP();
}
