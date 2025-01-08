/**
 * @file
 * @ingroup bsp_rgbled
 *
 * @brief  nRF52833-specific definition of the "rgb_led" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdint.h>
#include <string.h>
#include "gpio.h"
#include "rgbled.h"

//=========================== define ==========================================

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_NRF_SPIM (NRF_SPIM0_S)
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define DB_NRF_SPIM (NRF_SPIM0_NS)
#else
#define DB_NRF_SPIM (NRF_SPIM0)
#endif

// EasyDMA buffer size definition
#define LED_BUFFER_SIZE 32

// Define the ONE and ZERO enconding for the led one-wire protocol
#define LED_ONE  0b00010100
#define LED_ZERO 0b00010000

//=========================== variables =======================================

static const gpio_t _mosi_pin = { .port = 0, .pin = 3 };  ///< nRF52840 P0.3
static const gpio_t _sck_pin  = { .port = 1, .pin = 6 };  ///< nRF52840 P1.6 ( used because it's not an available pin in the BCM module).

// EasyDMA buffer declaration for the RGB LED.
typedef struct {
    uint8_t ledBuffer[LED_BUFFER_SIZE];
} rgbled_vars_t;

static rgbled_vars_t rgbled_vars;

//=========================== public ==========================================

void db_rgbled_init(void) {

    // Configure the necessary Pins in the GPIO peripheral, MOSI and SCK as Output
    db_gpio_init(&_mosi_pin, DB_GPIO_OUT);
    db_gpio_init(&_sck_pin, DB_GPIO_OUT);

    // Define the necessary Pins in the SPIM peripheral
    DB_NRF_SPIM->PSEL.MOSI = _mosi_pin.pin << SPIM_PSEL_MOSI_PIN_Pos |                        // Define pin number for MOSI pin
                             _mosi_pin.port << SPIM_PSEL_MOSI_PORT_Pos |                      // Define pin port for MOSI pin
                             SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos;  // Enable the MOSI pin

    DB_NRF_SPIM->PSEL.SCK = _sck_pin.pin << SPIM_PSEL_SCK_PIN_Pos |                        // Define pin number for SCK pin
                            _sck_pin.port << SPIM_PSEL_SCK_PORT_Pos |                      // Define pin port for SCK pin
                            SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos;  // Enable the SCK pin

    // Configure the SPIM peripheral
    DB_NRF_SPIM->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;                        // Set SPI frequency to 500Khz
    DB_NRF_SPIM->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first

    // Configure the WRITER EasyDMA channel
    DB_NRF_SPIM->TXD.MAXCNT = LED_BUFFER_SIZE;                   // Set the size of the output buffer.
    DB_NRF_SPIM->TXD.PTR    = (uint32_t)&rgbled_vars.ledBuffer;  // Set the output buffer pointer.

    // Enable the SPIM pripheral
    DB_NRF_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;

    // Execute a transfer
    DB_NRF_SPIM->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;  // trigger the SPI transfer
}

void db_rgbled_set(uint8_t red, uint8_t green, uint8_t blue) {

    // Make sure we update a cleared buffer
    memset(rgbled_vars.ledBuffer, 0, LED_BUFFER_SIZE);

    // Load the obligatory starting write command on to the buffer. This is required by the TLC5973 driver.
    rgbled_vars.ledBuffer[0] = 0x00;
    rgbled_vars.ledBuffer[1] = 0x84;
    rgbled_vars.ledBuffer[2] = 0x29;
    rgbled_vars.ledBuffer[3] = 0x4A;
    rgbled_vars.ledBuffer[4] = 0x42;
    rgbled_vars.ledBuffer[5] = 0x90;
    rgbled_vars.ledBuffer[6] = 0xA4;
    rgbled_vars.ledBuffer[7] = 0x29;
    rgbled_vars.ledBuffer[8] = 0x00;

    // ********** Load the Blue value into the buffer ************
    // Define the starting position of the blue color in the buffer.
    int buffer_byte = 8;
    int buffer_bit  = 59;

    // Iterate over all the bits of the blue color. (decreasing order, we write MSB first.)
    for (int i = 7; i >= 0; i--) {

        // Extract 1 bit from the blue color.
        int blue_bit = (blue >> i) & 0x01;
        // if the current bit is a one, copy the 5bits that represent a '1'. in the one-wire-protocol, on to the correct position in the buffer.
        if (blue_bit) {

            // iterate over all the bits in the 5bit encoded '1'.
            for (int j = 4; j >= 0; j--) {
                // Extract a single (j) bit from the '1' code, left shift it to the corect position, and write into the buffer.
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ONE >> j) & 0x01) << (buffer_bit % 8);
                // move the buffer bit position we are writting to, a single bit to the right.
                buffer_bit--;
                // if the next bit to write wraps over to the next byte, increment the buffer byte position.
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
        // However, if the current bit is a zero, repeat the above process but, with the 5bit encoded '0'
        else {
            for (int j = 4; j >= 0; j--) {
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ZERO >> j) & 0x01) << (buffer_bit % 8);
                buffer_bit--;
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
    }
    // fill the remaining 4 bytes of the 1-wire command with encoded zeros
    rgbled_vars.ledBuffer[13] |= 0x08;
    rgbled_vars.ledBuffer[14] = 0x42;
    rgbled_vars.ledBuffer[15] = 0x10;

    // ********** Load the Red value into the buffer ************
    // Define the starting position of the blue color in the buffer.
    buffer_byte = 16;
    buffer_bit  = 63;

    // Iterate over all the bits of the red color. (decreasing order, we write MSB first.)
    for (int i = 7; i >= 0; i--) {
        // Extract 1 bit from the red color.
        int red_bit = (red >> i) & 0x01;
        // if the current bit is a one, copy the 5bits that represent a '1'. in the one-wire-protocol, on to the correct position in the buffer.
        if (red_bit) {
            // iterate over all the bits in the 5bit encoded '1'.
            for (int j = 4; j >= 0; j--) {
                // Extract a single (j) bit from the '1' code, left shift it to the corect position, and write into the buffer.
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ONE >> j) & 0x01) << (buffer_bit % 8);
                // move the buffer bit position we are writting to, a single bit to the right.
                buffer_bit--;
                // if the next bit to write wraps over to the next byte, increment the buffer byte position.
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
        // However, if the current bit is a zero, repeat the above process but, with the 5bit encoded '0'
        else {
            for (int j = 4; j >= 0; j--) {
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ZERO >> j) & 0x01) << (buffer_bit % 8);
                buffer_bit--;
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
    }
    // fill the remaining 4 bytes of the 1-wire command with encoded zeros
    rgbled_vars.ledBuffer[21] = 0x84;
    rgbled_vars.ledBuffer[22] = 0x21;

    // ********** Load the green into the buffer ************
    // Define the starting position of the blue color in the buffer.
    buffer_byte = 23;
    buffer_bit  = 59;

    // Iterate over all the bits of the green color. (decreasing order, we write MSB first.)
    for (int i = 7; i >= 0; i--) {
        // Extract 1 bit from the green color.
        int green_bit = (green >> i) & 0x01;
        // if the current bit is a one, copy the 5bits that represent a '1'. in the one-wire-protocol, on to the correct position in the buffer.
        if (green_bit) {
            // iterate over all the bits in the 5bit encoded '1'.
            for (int j = 4; j >= 0; j--) {
                // Extract a single (j) bit from the '1' code, right left it to the corect position, and write into the buffer.
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ONE >> j) & 0x01) << (buffer_bit % 8);
                // move the buffer bit position we are writting to, a single bit to the right.
                buffer_bit--;
                // if the next bit to write wraps over to the next byte, increment the buffer byte position.
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
        // However, if the current bit is a zero, repeat the above process but, with the 5bit encoded '0'
        else {
            for (int j = 4; j >= 0; j--) {
                rgbled_vars.ledBuffer[buffer_byte] |= ((LED_ZERO >> j) & 0x01) << (buffer_bit % 8);
                buffer_bit--;
                if ((buffer_bit % 8) == 7) {
                    buffer_byte++;
                }
            }
        }
    }

    // fill the remaining 4 bytes of the 1-wire command with encoded zeros
    rgbled_vars.ledBuffer[28] |= 0x08;
    rgbled_vars.ledBuffer[29] = 0x42;
    rgbled_vars.ledBuffer[30] = 0x10;

    // Finally, execute the SPI transfer
    DB_NRF_SPIM->TASKS_START = SPIM_TASKS_START_TASKS_START_Trigger << SPIM_TASKS_START_TASKS_START_Pos;
}
