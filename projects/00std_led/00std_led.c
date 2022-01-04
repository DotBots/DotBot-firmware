/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
// #include "nrf52840_bitfields.h"
// #include "nrf52840.h"

// Pin definition
#define MOSI_PIN 3   // port 0
//#define MISO_PIN 4
#define SCK_PIN  6   // port 1  ( used because it's not an available pin in the BCM module)
//#define CS_PIN   29

// EasyDMA buffer definition
#define READERBUFFER_SIZE 32
#define WRITERBUFFER_SIZE 32

uint8_t readerBuffer[READERBUFFER_SIZE];
uint8_t writerBuffer[WRITERBUFFER_SIZE] = {0x00, 0x84, 0x29, 0x4A, 0x42, 0x90, 0xA4, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0xA5, 0x29, 0x4A, 0x52, 0x94, 0xA5, 0x29, 0x48, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00};

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/


void led_rgb(uint8_t r, uint8_t g, uint8_t b);



int main(void)
{

  // ********** Test the Board ************
  NRF_P0->DIRSET = 1 << 20;  
  NRF_P0->OUTSET = 1 << 20;  
  //NRF_P0->OUTCLR = 1 << 14;  


// ********** Configure the necessary Pins in the GPIO peripheral ************

  NRF_P0->DIRSET = 1 << MOSI_PIN;  // MOSI as Output
  //NRF_P0->DIRCLR = 1 << MISO_PIN;  // MISO as Input
  NRF_P0->DIRSET = 1 << SCK_PIN;   // SCK  as Output
  //NRF_P0->DIRSET = 1 << CS_PIN;    // CS   as Output


// ********** Define the necessary Pins in the SPIM peripheral ************

  NRF_SPIM0->PSEL.MOSI =  MOSI_PIN << SPIM_PSEL_MOSI_PIN_Pos  |                           // Define pin number for MOSI pin
                          0        << SPIM_PSEL_MOSI_PORT_Pos |                           // Define pin port for MOSI pin
                          SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos; // Enable the MOSI pin

  NRF_SPIM0->PSEL.SCK =  SCK_PIN  << SPIM_PSEL_SCK_PIN_Pos  |                             // Define pin number for SCK pin
                         1        << SPIM_PSEL_SCK_PORT_Pos |                             // Define pin port for SCK pin
                         SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos;    // Enable the SCK pin

// ********** Configure the SPIM peripheral ************

  NRF_SPIM0->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;                        // Set SPI frequency to 500Khz
  NRF_SPIM0->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first


// ********** Configure the EasyDMA channel ************

  // Configuring the READER channel
  NRF_SPIM0->RXD.MAXCNT = READERBUFFER_SIZE;              // Set the size of the input buffer.
  NRF_SPIM0->RXD.PTR    = &readerBuffer;                  // Set the input buffer pointer.

  // Configure the WRITER channel
  NRF_SPIM0->TXD.MAXCNT = WRITERBUFFER_SIZE;              // Set the size of the input buffer.
  NRF_SPIM0->TXD.PTR    = &writerBuffer;                  // Set the input buffer pointer.


  // ********** Enable the SPIM pripheral ************

  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;  // Enable the SPI periperal.


  // ********** Execute a transfer ************

  NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;  // trigger the SPI transfer

  // one last instruction.
  NRF_P0->DIRSET = 1 << 14;
}


void led_rgb(uint8_t r, uint8_t g, uint8_t b){




}

/*************************** End of file ****************************/
