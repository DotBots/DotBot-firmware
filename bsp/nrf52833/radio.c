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

#include "clock.h"
#include "radio.h"

//=========================== defines ==========================================

#define NUMBER_OF_BYTES_IN_PACKET 32
#define RADIO_INTERRUPT_PRIORITY  1

// On-air radio addresses, these are completely arbitrary numbers.
#define RADIO_BASE_ADDRESS_0 0x12345678UL
#define RADIO_BASE_ADDRESS_1 0xFEDCBA98UL

//=========================== variables ========================================

typedef struct {
    uint8_t packet[NUMBER_OF_BYTES_IN_PACKET];     // Variable that stores the radio packets that arrives and the radio packets that are about to be sent.
    uint8_t rx_buffer[NUMBER_OF_BYTES_IN_PACKET];  // Intermediate variable to store an arriving packet before sending it to the callback function.

    radio_cb_t callback;  // Function pointer, stores the callback to use in the RADIO_Irq handler.
} radio_vars_t;

static radio_vars_t radio_vars = { 0 };

//========================== prototypes ========================================

static void radio_init_common(radio_cb_t callback);

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback) {

    // General configuration of the radio.
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 0dBm == 1mW Power output
    NRF_RADIO->MODE    = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);         // Use BLE 1Mbit/s protocol

    NRF_RADIO->PCNF1 = (NUMBER_OF_BYTES_IN_PACKET << RADIO_PCNF1_MAXLEN_Pos) |     // The Payload maximum size is 32 bytes
                       (NUMBER_OF_BYTES_IN_PACKET << RADIO_PCNF1_STATLEN_Pos) |    // Since the LENGHT field is not set, this specifies the lenght of the payload
                       (4UL << RADIO_PCNF1_BALEN_Pos) |                            // The base address is 4 Bytes long
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |     // Make the on air packet be little endian (this enables some useful features)
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);  // Disable the package whitening feature.

    // Configuring the on-air radio address
    NRF_RADIO->BASE0       = RADIO_BASE_ADDRESS_0;                                              // base address for prefix 0
    NRF_RADIO->BASE1       = RADIO_BASE_ADDRESS_1;                                              // base address for prefix 1-7
    NRF_RADIO->TXADDRESS   = 0UL;                                                               // set device address 0 to use when transmitting (must match RXADDRESSES)
    NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);  // receive from address 0 (must match TXADDRESSES)

    // Initialize Common Radio Configuration
    radio_init_common(callback);
}

void db_radio_init_lr(radio_cb_t callback) {

    // General configuration of the radio.
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 8dBm Power output
    NRF_RADIO->MODE    = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);        // Use Long Range 125 kbps modulation

    // Coded PHY (Long range)
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |
                       (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_TERMLEN_Pos) |
                       (2 << RADIO_PCNF0_CILEN_Pos) |
                       (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos);

    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (3 << RADIO_PCNF1_BALEN_Pos) |
                       (0 << RADIO_PCNF1_STATLEN_Pos) |
                       (NUMBER_OF_BYTES_IN_PACKET << RADIO_PCNF1_MAXLEN_Pos);

    // Configuring the on-air radio address
    NRF_RADIO->BASE0 = RADIO_BASE_ADDRESS_0;  // base address for prefix 0
    NRF_RADIO->BASE1 = RADIO_BASE_ADDRESS_1;  // base address for prefix 1-7

    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
    NRF_RADIO->TXADDRESS   = (0 << RADIO_TXADDRESS_TXADDRESS_Pos) & RADIO_TXADDRESS_TXADDRESS_Msk;

    // Initialize Common Radio Configuration
    radio_init_common(callback);
}

void db_radio_set_frequency(uint8_t freq) {

    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

void db_radio_tx(uint8_t *tx_buffer, uint8_t length) {

    // Load the tx_buffer into memory.
    memcpy(radio_vars.packet, tx_buffer, length);

    // Configure the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  // yeet the packet as soon as the radio is ready - slow startup transmitters are for nerds, scumdog for l4f3
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);   // Disable the radio as soon as the packet is sent

    // Activate the RADIO and send the package
    NRF_RADIO->EVENTS_DISABLED = 0;                                    // Clear the flag before starting the radio.
    NRF_RADIO->TASKS_TXEN      = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger;  // Enable the Radio and let the shortcuts deal with all the
                                                                       // steps to send the packet and disable the radio
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}                         // Wait for the radio to actually send the package.

    // Clear the packet.
    memset(radio_vars.packet, 0, NUMBER_OF_BYTES_IN_PACKET);
}

void db_radio_rx_enable(void) {

    // Configure the Shortcuts to expedite the packet reception.
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_START_Enabled << RADIO_SHORTS_END_START_Pos);

    // Start the Radio for reception
    NRF_RADIO->EVENTS_RXREADY = 0;                                    // Clear the flag before enabling the Radio.
    NRF_RADIO->TASKS_RXEN     = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;  // Enable radio reception.
    while (NRF_RADIO->EVENTS_RXREADY == 0) {}                         // Wait for the radio to actually start receiving.

    // Enable Radio interruptions
    NVIC_EnableIRQ(RADIO_IRQn);
}

void db_radio_rx_disable(void) {

    NRF_RADIO->EVENTS_DISABLED = 0;                                          // Clear the flag before starting the radio.
    NRF_RADIO->TASKS_DISABLE   = RADIO_TASKS_DISABLE_TASKS_DISABLE_Trigger;  // Disable radio reception.
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}                               // Wait for the radio to actually disable itself.

    // Disable Radio interruptions
    NVIC_DisableIRQ(RADIO_IRQn);
}

//=========================== private ==========================================

/**
 * @brief This function is private and it sets the common configurations for the radio
 *
 * @param[in] callback pointer to a function that will be called each time a packet is received.
 */
void radio_init_common(radio_cb_t callback) {

    // CRC Config
    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos);  // Checksum uses 2 bytes, and is enabled.
    NRF_RADIO->CRCINIT = 0xFFFFUL;                                        // initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;                                       // CRC poly: x^16 + x^12^x^5 + 1

    // pointer to packet payload
    NRF_RADIO->PACKETPTR = (uint32_t)radio_vars.packet;

    // Assign the callback function that will be called when a radio packet is received.
    radio_vars.callback = callback;

    // Configure the external High-frequency Clock. (Needed for correct operation)
    db_hfclk_init();

    // Configure the Interruptions
    NVIC_DisableIRQ(RADIO_IRQn);  // Disable interruptions while configuring
    NRF_RADIO->INTENSET = (RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos |
                           RADIO_INTENSET_CRCOK_Enabled << RADIO_INTENSET_CRCOK_Pos);  // Enable interruption for when a valid packet arrives
    NVIC_SetPriority(RADIO_IRQn, RADIO_INTERRUPT_PRIORITY);                            // Set priority for Radio interrupts to 1
    NVIC_ClearPendingIRQ(RADIO_IRQn);                                                  // Clear the flag for any pending radio interrupt
}

//=========================== interrupt handlers ===============================

/**
 * @brief Interruption handler for the Radio.
 *
 * This function will be called each time a radio packet is received.
 * it will clear the interrupt, copy the last received packet
 * and called the user-defined callback to process the package.
 *
 */
void RADIO_IRQHandler(void) {

    NVIC_ClearPendingIRQ(RADIO_IRQn);

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
    }

    // Check if the interrupt was caused by a fully received package
    if (NRF_RADIO->EVENTS_CRCOK) {

        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_CRCOK = 0;

        if (radio_vars.callback) {
            // Copy packet into the buffer, before sending it to the callback.
            memcpy(radio_vars.rx_buffer, radio_vars.packet, NUMBER_OF_BYTES_IN_PACKET);

            // Call callback defined by user.
            radio_vars.callback(radio_vars.rx_buffer, NUMBER_OF_BYTES_IN_PACKET);

            // Clear the rx_buffer.
            memset(radio_vars.rx_buffer, 0, NUMBER_OF_BYTES_IN_PACKET);
        }
    }
}
