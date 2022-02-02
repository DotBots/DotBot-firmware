/*********************************************************************
*                                                                    * 
*                               DotBot                               *
*                                                                    *
**********************************************************************/

/**
 * @file 00std_led.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the RGB LED in the DotBot board.
 * 
 * Load this program on your board. The LEDs should start blinking different colors.
 * 
 * @date 2022
 * 
 * @copyright Inria, 2022
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== defines =========================================

// Pin definitions
#define MOSI_PIN 3   ///< nRF52840 P0.3
#define SCK_PIN 6    ///< nRF52840 P1.6 (used because it's not an available pin in the BCM module).
// Note, the SPIM peripheral refuses to enable itself if the SCK pin is not defined.

// EasyDMA buffer size definition
#define READERBUFFER_SIZE 32
#define WRITERBUFFER_SIZE 32

// Define a blocking wait function.
#define WAIT_MS(MS) for (int i = 0; i < 3000 * MS; i++) {;}

//=========================== variables =========================================
// EasyDMA READ buffer declaration (the read buffer is not actually required for the code to work. 4
// But is left as an example of how to configure an EasyDMA READ buffer.)
uint8_t readerBuffer[READERBUFFER_SIZE];

// Various EasyDMA WRITE buffers preloaded with the dataframes required for displaying different colors on the LED.
// When these dataframes are shifted out (MSB first) to the LED driver at 500Kbits/s.
// the LED driver recognizes them as one-wire commands for the pre-specified colors.
// And turns ON the RGB LED accordingly.
uint8_t led_cyan_buffer[WRITERBUFFER_SIZE]    = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x00};
uint8_t led_blue_buffer[WRITERBUFFER_SIZE]    = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x52, 0x90, 0x85, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00};
uint8_t led_magenta_buffer[WRITERBUFFER_SIZE] = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x84, 0x29, 0x48, 0x42, 0x90, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00};
uint8_t led_red_buffer[WRITERBUFFER_SIZE]     = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x85, 0x29, 0x08, 0x52, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00};
uint8_t led_yellow_buffer[WRITERBUFFER_SIZE]  = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x29, 0x48, 0x42, 0x90, 0x84, 0x21, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x00};
uint8_t led_green_buffer[WRITERBUFFER_SIZE]   = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x52, 0x90, 0x85, 0x21, 0x08, 0x42, 0x10, 0x00};
uint8_t led_white_buffer[WRITERBUFFER_SIZE]   = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x84, 0x29, 0x48, 0x42, 0x90, 0x84, 0x21, 0x08, 0x42, 0x94, 0x84, 0x29, 0x08, 0x42, 0x10, 0x00};
uint8_t led_black_buffer[WRITERBUFFER_SIZE]   = {0x00, 0x84, 0x29, 0x4a, 0x42, 0x90, 0xa4, 0x29, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x84, 0x21, 0x08, 0x42, 0x10, 0x00};

//=========================== main =========================================

/**
*  @brief The program starts executing here.
*/
int main(void)
{

  // Turn ON the DotBot board regulator 
  NRF_P0->DIRSET = 1 << 20;  
  NRF_P0->OUTSET = 1 << 20;   

  // Configure the necessary Pins in the GPIO peripheral  (MISO and CS not needed)
  NRF_P0->DIRSET = 1 << MOSI_PIN;  // MOSI as Output
  NRF_P0->DIRSET = 1 << SCK_PIN;   // SCK  as Output

  // Define the necessary Pins in the SPIM peripheral 
  NRF_SPIM0->PSEL.MOSI =  MOSI_PIN << SPIM_PSEL_MOSI_PIN_Pos  |                           // Define pin number for MOSI pin
                          0        << SPIM_PSEL_MOSI_PORT_Pos |                           // Define pin port for MOSI pin
                          SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos; // Enable the MOSI pin

  NRF_SPIM0->PSEL.SCK =  SCK_PIN  << SPIM_PSEL_SCK_PIN_Pos  |                             // Define pin number for SCK pin
                         1        << SPIM_PSEL_SCK_PORT_Pos |                             // Define pin port for SCK pin
                         SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos;    // Enable the SCK pin

  // Configure the SPIM peripheral 
  NRF_SPIM0->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;                        // Set SPI frequency to 500Khz
  NRF_SPIM0->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first

  // Configure the EasyDMA channel 
  // Configuring the READER channel
  NRF_SPIM0->RXD.MAXCNT = READERBUFFER_SIZE;              // Set the size of the input buffer.
  NRF_SPIM0->RXD.PTR    = &readerBuffer;                  // Set the input buffer pointer.
  // Configure the WRITER channel
  NRF_SPIM0->TXD.MAXCNT = WRITERBUFFER_SIZE;              // Set the size of the output buffer.

  // Enable the SPIM pripheral 
  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;

  while (1)
  {
    // Set color of the LED to Cyan
    NRF_SPIM0->TXD.PTR = &led_cyan_buffer;
    // Execute a transfer 
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    // Wait a couple of seconds to appreciate the led color before changing to the next.
    WAIT_MS(2000);

    // Set color of the LED to Blue
    NRF_SPIM0->TXD.PTR = &led_blue_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to Magenta
    NRF_SPIM0->TXD.PTR = &led_magenta_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to Red
    NRF_SPIM0->TXD.PTR = &led_red_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to Yellow
    NRF_SPIM0->TXD.PTR = &led_yellow_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to Green
    NRF_SPIM0->TXD.PTR = &led_green_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to White
    NRF_SPIM0->TXD.PTR = &led_white_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);

    // Set color of the LED to Black
    NRF_SPIM0->TXD.PTR = &led_black_buffer;
    NRF_SPIM0->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
    WAIT_MS(2000);
  }

  // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
  __NOP();
}
