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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "clock.h"
#include "radio.h"

//=========================== defines ==========================================

#define PAYLOAD_MAX_LENGTH       UINT8_MAX
#define PACKET_MAX_LENGTH        (PAYLOAD_MAX_LENGTH + 2)
#define RADIO_INTERRUPT_PRIORITY 1

typedef struct {
    uint8_t header;                       ///< PDU header (depends on the type of PDU - advertising physical channel or Data physical channel)
    uint8_t length;                       ///< Length of the payload + MIC (if any)
    uint8_t payload[PAYLOAD_MAX_LENGTH];  ///< Payload + MIC (if any)
} ble_radio_pdu_t;

typedef struct {
    ble_radio_pdu_t pdu;       ///< Variable that stores the radio PDU (protocol data unit) that arrives and the radio packets that are about to be sent.
    radio_cb_t      callback;  ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
} radio_vars_t;

//=========================== variables ========================================

static const uint8_t _chan_to_freq[40] = {
    4, 6, 8,
    10, 12, 14, 16, 18,
    20, 22, 24, 28,
    30, 32, 34, 36, 38,
    40, 42, 44, 46, 48,
    50, 52, 54, 56, 58,
    60, 62, 64, 66, 68,
    70, 72, 74, 76, 78,
    2, 26, 80   // Advertising channels
};

static radio_vars_t radio_vars = { 0 };

//========================== prototypes ========================================

static void radio_init_common(radio_cb_t callback);
static void radio_init_addresses(void);

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback) {

    // General configuration of the radio.
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 0dBm == 1mW Power output
    NRF_RADIO->MODE    = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);         // Use BLE 1Mbit/s protocol

    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |                    // S1 field length in bits
                       (1 << RADIO_PCNF0_S0LEN_Pos) |                    // S0 field length in bytes
                       (8 << RADIO_PCNF0_LFLEN_Pos) |                    // LENGTH field length in bits
                       (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);  // PREAMBLE length is 1 byte in BLE 1Mbit/s and 2Mbit/s

    NRF_RADIO->PCNF1 = (4UL << RADIO_PCNF1_BALEN_Pos) |  // The base address is 4 Bytes long
                       (PACKET_MAX_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |
                       (0 << RADIO_PCNF1_STATLEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |     // Make the on air packet be little endian (this enables some useful features)
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);  // Disable the package whitening feature.

    radio_init_addresses();

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
                       (PACKET_MAX_LENGTH << RADIO_PCNF1_MAXLEN_Pos);

    radio_init_addresses();

    // Initialize Common Radio Configuration
    radio_init_common(callback);
}

void db_radio_set_frequency(uint8_t freq) {

    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

void db_radio_set_channel(uint8_t channel) {
    NRF_RADIO->FREQUENCY = (_chan_to_freq[channel] << RADIO_FREQUENCY_FREQUENCY_Pos);
}

void db_radio_set_network_address(uint32_t addr) {
    NRF_RADIO->BASE0 = addr;
}

void db_radio_tx(uint8_t *tx_buffer, uint8_t length) {

    // Load the tx_buffer into memory.
    radio_vars.pdu.length = length;
    memcpy(radio_vars.pdu.payload, tx_buffer, length);

    // Configure the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  // yeet the packet as soon as the radio is ready - slow startup transmitters are for nerds, scumdog for l4f3
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);   // Disable the radio as soon as the packet is sent

    // Activate the RADIO and send the package
    NRF_RADIO->EVENTS_DISABLED = 0;                                    // Clear the flag before starting the radio.
    NRF_RADIO->TASKS_TXEN      = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger;  // Enable the Radio and let the shortcuts deal with all the
                                                                       // steps to send the packet and disable the radio
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}                         // Wait for the radio to actually send the package.
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

static void radio_init_addresses(void) {
    // Configuring the on-air radio address.
    NRF_RADIO->BASE0 = DEFAULT_NETWORK_ADDRESS;

    // only send using logical address 0
    NRF_RADIO->TXADDRESS = 0UL;
    // only receive from logical address 0
    NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
}

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
    NRF_RADIO->PACKETPTR = (uint32_t)&radio_vars.pdu;

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

    // Check if the interrupt was caused by a fully received package
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
    }

    // Check if the interrupt was caused by a fully received package
    if (NRF_RADIO->EVENTS_CRCOK) {

        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_CRCOK = 0;

        if (radio_vars.callback) {
            // Call callback defined by user.
            radio_vars.callback(radio_vars.pdu.payload, radio_vars.pdu.length);
        }
    }
}
