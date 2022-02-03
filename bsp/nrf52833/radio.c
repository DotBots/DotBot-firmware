/**
 * @file radio.c
 * @addtogroup BSP
 * 
 * @brief  nRF52833-specific definition of the "radio" bsp module.
 * 
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * 
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "radio.h"

//=========================== defines =========================================

// Simple function that blocks and wait while a flag sets.
#define WAIT_FOR(FLAG) while (FLAG == 0) {__NOP();}                  

#define MAX_NUMBER_OF_BYTES_IN_PACKET 32

//=========================== variables =========================================

//=========================== public ==========================================

/**
 * @brief Configures the RADIO peripheral to work as a transmitter
 * 
 * The Radio is configured in the following way:
 * 
 * Radio frequency = 2400Mhz + [0,100]MHz 
 * TX Power = 0dBm
 * Packet size = 32bytes
 * 
 * @param[in] freq Frequency of the Radio, by the following formula = 2400 + freq (MHz) [0, 100]
 * @param[in] log_addr Logial addres that the radio will use [0, 7]
 * @param[in] packet Pointer to a uint_8 array of size 32 where the packet will be located.
 * 
 */
void db_radio_tx_init(uint8_t freq, uint8_t log_addr, uint8_t *packet)
{
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos); // 0dBm == 1mW Power output
    NRF_RADIO->FREQUENCY = freq;                                                     // frequency bin 7, 2407MHz
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);           // Use Nordic proprietary 1Mbit/s protocol

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

    NRF_RADIO->TXADDRESS = log_addr; // // Transmit to this address

    // packet configuration
    NRF_RADIO->PCNF0 = 0UL; //not really interested in these

    NRF_RADIO->PCNF1 = (32UL << RADIO_PCNF1_MAXLEN_Pos) |                         // The Payload maximum size is 32 bytes
                       (32UL << RADIO_PCNF1_STATLEN_Pos) |                        // since the LENGHT field is not set, this specifies the lenght of the payload
                       (4UL << RADIO_PCNF1_BALEN_Pos) |                           // The base address is 4 Bytes long
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |    // Make the on air packet be little endian (this enables some useful features)
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos); // Disable the package whitening feature.

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Checksum uses 2 bytes, and is enabled.
    NRF_RADIO->CRCINIT = 0xFFFFUL;                                      // initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;                                     // CRC poly: x^16 + x^12^x^5 + 1

    // pointer to packet payload
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | // yeet the packet as soon as the radio is ready - slow startup transmitters are for nerds, scumdog for l4f3
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);  //FIXME: this short does not work, radio is not disabled upon "END" of transmission

    // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;    // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART = 0x01;       // Start the clock
    WAIT_FOR(NRF_CLOCK->EVENTS_HFCLKSTARTED); // Wait for the clock to actually start.
}

/**
 * @brief Configures the RADIO peripheral to work as a receiver, and starts receiving.
 * 
 * After this function is called, the "packet" array will be constantly updated with 
 * arriving packets.
 * The Radio is configured in the following way:
 * 
 * Radio frequency = 2400Mhz + [0,100]MHz 
 * TX Power = 0dBm
 * Packet size = 32bytes
 * 
 * @param[in] freq Frequency of the radio, by the following formula = 2400 + freq (MHz) [0, 100]
 * @param[in] log_addr Logical addres that the radio will use. [0, 7]
 * @param[in] packet Pointer to a uint_8 array of size 32 where the packet will be located.
 * 
 */
void db_radio_rx_init(uint8_t freq, uint8_t log_addr, uint8_t packet[32])
{
    NRF_RADIO->FREQUENCY = freq;                                          // frequency bin 7, 2407MHz
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

    NRF_RADIO->RXADDRESSES = (1UL << log_addr); // Receive from this address

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
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    // // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;    // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART = 0x01;       // Start the clock
    WAIT_FOR(NRF_CLOCK->EVENTS_HFCLKSTARTED); // Wait for the clock to actually start.

    // Enable the Radio
    NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;
}

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 * 
 * Radio frequency 2400 + freq (MHz) [0, 100]
 * 
 * @param[in] freq Frequency of the radio [0, 100]
 */
void db_radio_set_frequency(uint8_t freq)
{
    NRF_RADIO->FREQUENCY = freq; // frequency bin 7, 2407MHz
}

/**
 * @brief Set the tx-rx logical address of the radio.
 * 
 * Selects one of the 8 predetermined HW address of the radio.
 * 
 * @param[in] log_addr Logical addres that the radio will use. [0, 7]
 */
void db_radio_set_logical_address(uint8_t log_addr)
{
    NRF_RADIO->RXADDRESSES = (1UL << log_addr); // Receive from this address
    NRF_RADIO->TXADDRESS = log_addr;            // Transmit to this address
}

/**
 * @brief Sends a packet through the Radio.
 * 
 * Grabs the preselected packet chosen in the tx_innit function
 * and sends it through the radio.
 * 
 */
void db_radio_send_packet(void)
{
    NRF_RADIO->TASKS_TXEN = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger; // Enable the Radio and let the shortcuts deal with all the 
                                                                 // steps to send the packet and disable the radio
}

/**
 * @brief Starts Receiving packets thtough the Radio.
 * 
 * 
 */
void db_radio_rx_enable(void)
{
    NRF_RADIO->TASKS_RXEN = 1UL;
}

/**
 * @brief Stops Receiving packets thtough the Radio.
 * 
 * 
 */
void db_radio_rx_disable(void)
{
    NRF_RADIO->TASKS_DISABLE = RADIO_TASKS_DISABLE_TASKS_DISABLE_Trigger;
}