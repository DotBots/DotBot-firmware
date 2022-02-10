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
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radio.h"

//=========================== defines =========================================

#define NUMBER_OF_BYTES_IN_PACKET 32
#define RADIO_INTERRUPT_PRIORITY  1

//=========================== variables =========================================

static uint8_t packet[NUMBER_OF_BYTES_IN_PACKET]; // Variable that stores the radio packet that arrives.

void (*RADIO_callback)(void) = NULL;              // Function pointer, stores the callback to use in the RADIO_Irq handler.
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
void db_radio_init(void (*callback)()) {

    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos); // 0dBm == 1mW Power output
    NRF_RADIO->FREQUENCY = 8 << RADIO_FREQUENCY_FREQUENCY_Pos;                        // frequency bin 8, 2408MHz
    NRF_RADIO->MODE      = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);        // Use BLE 1Mbit/s protocol

    NRF_RADIO->PCNF1 = (NUMBER_OF_BYTES_IN_PACKET << RADIO_PCNF1_MAXLEN_Pos)  |     // The Payload maximum size is 32 bytes
                       (NUMBER_OF_BYTES_IN_PACKET << RADIO_PCNF1_STATLEN_Pos) |     // Since the LENGHT field is not set, this specifies the lenght of the payload
                       (4UL << RADIO_PCNF1_BALEN_Pos) |                             // The base address is 4 Bytes long
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |      // Make the on air packet be little endian (this enables some useful features)
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);   // Disable the package whitening feature.

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos);             // Checksum uses 2 bytes, and is enabled.
    NRF_RADIO->CRCINIT = 0xFFFFUL;                                                  // initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;                                                 // CRC poly: x^16 + x^12^x^5 + 1

    // pointer to packet payload
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;                                          // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;                                          // Start the clock
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {;}                                 // Wait for the clock to actually start.

    // Configure the callbacks
    RADIO_callback = callback;

    // Configure the Interruptions
    NVIC_DisableIRQ(RADIO_IRQn);                                                    // Disable interruptions while configuring

    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos;     // Enable interruption for when a packet arrives
    NVIC_SetPriority(RADIO_IRQn, RADIO_INTERRUPT_PRIORITY);                         // Set priority for Radio interrupts to 1
    NVIC_ClearPendingIRQ(RADIO_IRQn);                                               // Clear the flag for any pending radio interrupt 

    // NVIC_EnableIRQ(RADIO_IRQn);                                                     // Config done, reenable interruptions.
}

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 *
 * Radio frequency 2400 + freq (MHz) [0, 100]
 *
 * @param[in] freq Frequency of the radio [0, 100]
 */
void db_radio_set_frequency(uint8_t freq) {

    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

/**
 * @brief Sends a packet through the Radio.
 *
 * Send a package over the rado
 *
 * @param[in] package pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length  Number of bytes to send (max size = 32)
 *
 */
void db_radio_tx_send(uint8_t *package, uint8_t length) {

    // Load the package into memory.
    memcpy(packet, package, length);

    // Configure the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | // yeet the packet as soon as the radio is ready - slow startup transmitters are for nerds, scumdog for l4f3
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);  // Disable the radio as soon as the packet is sent

    // Activate the RADIO and send the package
    NRF_RADIO->EVENTS_DISABLED = 0;                                 // Clear the flag before starting the radio.
    NRF_RADIO->TASKS_TXEN = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger;    // Enable the Radio and let the shortcuts deal with all the
                                                                    // steps to send the packet and disable the radio
    while (NRF_RADIO->EVENTS_DISABLED == 0) {;}                     // Wait for the radio to actually send the package.

    // Clear the packet.
    memset(packet, 0, NUMBER_OF_BYTES_IN_PACKET);
}

/**
 * @brief Starts Receiving packets thtough the Radio.
 *
 */
void db_radio_rx_enable(void) {

    // Configure the Short to expedite the packet reception.
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos);   // yeet the packet as soon as the radio is ready - slow startup transmitters are for nerds, scumdog for l4f3

    // Start the Radio for reception
    NRF_RADIO->EVENTS_RXREADY = 0;                                  // Clear the flag before enabling the Radio.
    NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;    // Enable radio reception.
    while (NRF_RADIO->EVENTS_RXREADY == 0) {;}                      // Wait for the radio to actually start receiving.

    // Enable Radio interruptions
    NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * @brief Stops receiving packets thtough the Radio.
 *
 */
void db_radio_rx_disable(void) {

    NRF_RADIO->EVENTS_DISABLED = 0;                                         // Clear the flag before starting the radio.
    NRF_RADIO->TASKS_DISABLE = RADIO_TASKS_DISABLE_TASKS_DISABLE_Trigger;   // Disable radio reception. 
    while (NRF_RADIO->EVENTS_DISABLED == 0) {;}                             // Wait for the radio to actually disable itself.

    // Disable Radio interruptions
    NVIC_DisableIRQ(RADIO_IRQn);
}


//=========================== interrupt handlers ==============================

void RADIO_IRQHandler(void) {

    // Check if the interrupt was caused by a fully received package
    if (NRF_RADIO->EVENTS_END) {

        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_END = 0;      

        // Call callback defined by user. 
        (*RADIO_callback)();
    }

}