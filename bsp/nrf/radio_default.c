/**
 * @file
 * @ingroup bsp_radio
 *
 * @brief  nRF52833-specific definition of the "radio" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 *
 * @copyright Inria, 2022-2024
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "clock.h"
#include "radio.h"

//=========================== defines ==========================================

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define NRF_RADIO NRF_RADIO_NS
#endif

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define RADIO_INTERRUPT_PRIORITY 2
#else
#define RADIO_INTERRUPT_PRIORITY 1
#endif

#define RADIO_TIFS          0U  ///< Inter frame spacing in us. zero means IFS is enforced by software, not the hardware
#define RADIO_SHORTS_COMMON (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |                 \
                                (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |             \
                                (RADIO_SHORTS_ADDRESS_RSSISTART_Enabled << RADIO_SHORTS_ADDRESS_RSSISTART_Pos) | \
                                (RADIO_SHORTS_DISABLED_RSSISTOP_Enabled << RADIO_SHORTS_DISABLED_RSSISTOP_Pos)
#define RADIO_INTERRUPTS (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos) | \
                             (RADIO_INTENSET_ADDRESS_Enabled << RADIO_INTENSET_ADDRESS_Pos)
#define RADIO_STATE_IDLE 0x00
#define RADIO_STATE_RX   0x01
#define RADIO_STATE_TX   0x02
#define RADIO_STATE_BUSY 0x04

typedef struct __attribute__((packed)) {
    uint8_t header;                              ///< PDU header (depends on the type of PDU - advertising physical channel or Data physical channel)
    uint8_t length;                              ///< Length of the payload + MIC (if any)
    uint8_t payload[DB_BLE_PAYLOAD_MAX_LENGTH];  ///< Payload + MIC (if any) (DB_BLE_PAYLOAD_MAX_LENGTH > DB_IEEE802154_PAYLOAD_MAX_LENGTH)
} radio_pdu_t;

typedef struct {
    radio_pdu_t     pdu;       ///< Variable that stores the radio PDU (protocol data unit) that arrives and the radio packets that are about to be sent.
    radio_cb_t      callback;  ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    uint8_t         state;     ///< Internal state of the radio
    db_radio_mode_t mode;      ///< PHY protocol used by the radio (BLE, IEEE 802.15.4)
} radio_vars_t;

//=========================== variables ========================================

static const uint8_t _ble_chan_to_freq[40] = {
    4, 6, 8,
    10, 12, 14, 16, 18,
    20, 22, 24, 28,
    30, 32, 34, 36, 38,
    40, 42, 44, 46, 48,
    50, 52, 54, 56, 58,
    60, 62, 64, 66, 68,
    70, 72, 74, 76, 78,
    2, 26, 80  // Advertising channels
};

static radio_vars_t radio_vars = { 0 };

//========================== prototypes ========================================

static void _radio_enable(void);

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback, db_radio_mode_t mode) {

#if defined(NRF5340_XXAA)
    // On nrf53 configure constant latency mode for better performances
    NRF_POWER_NS->TASKS_CONSTLAT = 1;
#endif

    // Reset radio to its initial values
    NRF_RADIO->POWER = (RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos);
    NRF_RADIO->POWER = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);

#if defined(NRF5340_XXAA)
    // Copy all the RADIO trim values from FICR into the target addresses (from errata v1.6 - 3.29 [158])
    for (uint32_t index = 0; index < 32ul && NRF_FICR_NS->TRIMCNF[index].ADDR != (uint32_t *)0xFFFFFFFFul; index++) {
        if (((uint32_t)NRF_FICR_NS->TRIMCNF[index].ADDR & 0xFFFFF000ul) == (volatile uint32_t)NRF_RADIO_NS) {
            *((volatile uint32_t *)NRF_FICR_NS->TRIMCNF[index].ADDR) = NRF_FICR_NS->TRIMCNF[index].DATA;
        }
    }
#endif

    // General configuration of the radio.
    radio_vars.mode = mode;  // Set global radio mode
    if (mode == DB_RADIO_IEEE802154_250Kbit) {
        // Configure IEEE 802.15.4 mode
        NRF_RADIO->MODE = (RADIO_MODE_MODE_Ieee802154_250Kbit << RADIO_MODE_MODE_Pos);
    } else {
        // Configure BLE modes (e.g., BLE 1Mbit, 2Mbit, etc.)
        NRF_RADIO->MODE = ((RADIO_MODE_MODE_Ble_1Mbit + mode) << RADIO_MODE_MODE_Pos);
    }

#if defined(NRF5340_XXAA)
    // From errata v1.6 - 3.15 [117] RADIO: Changing MODE requires additional configuration
    if (mode == DB_RADIO_BLE_2MBit) {
        *((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0084);
    } else {
        *((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0080);
    }
#endif
    // Packet configuration of the radio
    if (mode == DB_RADIO_IEEE802154_250Kbit) {
        NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  // Set transmission power to 0dBm

        // Packet configuration register 0
        NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |                          // S1 field length in bits
                           (0 << RADIO_PCNF0_S0LEN_Pos) |                          // S0 field length in bytes
                           (8 << RADIO_PCNF0_LFLEN_Pos) |                          // 8-bit length field
                           (RADIO_PCNF0_PLEN_32bitZero << RADIO_PCNF0_PLEN_Pos) |  // 4 bytes that are all zero for IEEE 802.15.4
                           (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);

        // // Packet configuration register 1
        NRF_RADIO->PCNF1 = (DB_IEEE802154_PAYLOAD_MAX_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |  // Max payload of 127 bytes
                           (0 << RADIO_PCNF1_STATLEN_Pos) |                                // 0 bytes added to payload length
                           (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |         // Little-endian format
                           (0 << RADIO_PCNF1_BALEN_Pos) |                                  // Base address length
                           (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);       // Enable whitening

    } else if (mode == DB_RADIO_BLE_1MBit || mode == DB_RADIO_BLE_2MBit) {
        NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 0dBm == 1mW Power output
        NRF_RADIO->PCNF0   = (0 << RADIO_PCNF0_S1LEN_Pos) |                              // S1 field length in bits
                           (1 << RADIO_PCNF0_S0LEN_Pos) |                                // S0 field length in bytes
                           (8 << RADIO_PCNF0_LFLEN_Pos) |                                // LENGTH field length in bits
                           (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);              // PREAMBLE length is 1 byte in BLE 1Mbit/s and 2Mbit/s

        NRF_RADIO->PCNF1 = (4UL << RADIO_PCNF1_BALEN_Pos) |  // The base address is 4 Bytes long
                           (DB_BLE_PAYLOAD_MAX_LENGTH << RADIO_PCNF1_MAXLEN_Pos) |
                           (0 << RADIO_PCNF1_STATLEN_Pos) |
                           (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |    // Make the on air packet be little endian (this enables some useful features)
                           (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);  // Enable data whitening feature.

    } else {  // Long ranges modes (125KBit/500KBit)
#if defined(NRF5340_XXAA)
        NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 0dBm Power output
#else
        NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);  // 8dBm Power output
#endif

        // Coded PHY (Long Range)
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
                           (DB_BLE_PAYLOAD_MAX_LENGTH << RADIO_PCNF1_MAXLEN_Pos);
    }

    // Address configuration
    NRF_RADIO->BASE0       = DEFAULT_NETWORK_ADDRESS;                                           // Configuring the on-air radio address
    NRF_RADIO->TXADDRESS   = 0UL;                                                               // Only send using logical address 0
    NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);  // Only receive from logical address 0

    // Inter frame spacing in us
    NRF_RADIO->TIFS = RADIO_TIFS;

    // Enable Fast TX Ramp Up
    NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos) |
                          (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);

    // CRC Config
    if (mode == DB_RADIO_IEEE802154_250Kbit) {
        NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |                  // 16-bit (2 bytes) CRC
                            (RADIO_CRCCNF_SKIPADDR_Ieee802154 << RADIO_CRCCNF_SKIPADDR_Pos);  // CRCCNF = 0x202 for IEEE 802.15.4
        NRF_RADIO->CRCINIT = 0;                                                               // The start value used by IEEE 802.15.4 is zero
        NRF_RADIO->CRCPOLY = 0x11021;
    } else {
        NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);  // Checksum uses 3 bytes, and is enabled.
        NRF_RADIO->CRCINIT = 0xFFFFUL;                                                                                                      // initial value
        NRF_RADIO->CRCPOLY = 0x00065b;                                                                                                      // CRC poly: x^16 + x^12^x^5 + 1
    }

    // Configure pointer to PDU for EasyDMA
    if (mode == DB_RADIO_IEEE802154_250Kbit) {
        NRF_RADIO->PACKETPTR = (uint32_t)((uint8_t *)&radio_vars.pdu + 1);  // Skip header for IEEE 802.15.4
    } else {
        NRF_RADIO->PACKETPTR = (uint32_t)&radio_vars.pdu;
    }

    // Assign the callback function that will be called when a radio packet is received.
    radio_vars.callback = callback;
    radio_vars.state    = RADIO_STATE_IDLE;

    // Configure the external High-frequency Clock. (Needed for correct operation)
    db_hfclk_init();

    // Configure the Interruptions
    NVIC_SetPriority(RADIO_IRQn, RADIO_INTERRUPT_PRIORITY);  // Set priority for Radio interrupts to 1
    // Clear all radio interruptions
    NRF_RADIO->INTENCLR = 0xffffffff;
    NVIC_EnableIRQ(RADIO_IRQn);
}

void db_radio_set_frequency(uint8_t freq) {
    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

void db_radio_set_channel(uint8_t channel) {
    uint8_t freq;
    if (radio_vars.mode == DB_RADIO_IEEE802154_250Kbit) {
        assert(channel >= 11 && channel <= 26 && "Channel value must be between 11 and 26 for IEEE 802.15.4");
        freq = 5 * (channel - 10);  // Frequency offset in MHz from 2400 MHz
    } else {
        freq = _ble_chan_to_freq[channel];
    }

    db_radio_set_frequency(freq);
}

void db_radio_set_network_address(uint32_t addr) {
    NRF_RADIO->BASE0 = addr;
}

void db_radio_tx(const uint8_t *tx_buffer, uint8_t length) {
    radio_vars.pdu.length = length;
    memcpy(radio_vars.pdu.payload, tx_buffer, length);

    NRF_RADIO->SHORTS = RADIO_SHORTS_COMMON | (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);

    if (radio_vars.state == RADIO_STATE_IDLE) {

        // Enable the Radio to send the packet
        NRF_RADIO->EVENTS_DISABLED = 0;  // We must use EVENT_DISABLED, if we use EVENT_END. the interrupts will be enabled in the time between the END event and the Disable event, triggering an undesired interrupt
        NRF_RADIO->TASKS_TXEN      = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger << RADIO_TASKS_TXEN_TASKS_TXEN_Pos;
        // Wait for transmission to end and the radio to be disabled
        while (NRF_RADIO->EVENTS_DISABLED == 0) {}

        // We re-enable interrupts AFTER the packet is sent, to avoid triggering an EVENT_ADDRESS and EVENT_DISABLED interrupt with the outgoing packet
        // We also clear both flags to avoid insta-triggering an interrupt as soon as we assert INTENSET
        NRF_RADIO->EVENTS_ADDRESS  = 0;
        NRF_RADIO->EVENTS_DISABLED = 0;
        _radio_enable();
    }
    radio_vars.state = RADIO_STATE_RX;
}

void db_radio_rx(void) {
    NRF_RADIO->SHORTS   = RADIO_SHORTS_COMMON | (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    NRF_RADIO->INTENSET = RADIO_INTERRUPTS;

    if (radio_vars.state == RADIO_STATE_IDLE) {
        _radio_enable();
        NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;
    }
    radio_vars.state = RADIO_STATE_RX;
}

void db_radio_disable(void) {
    NRF_RADIO->INTENCLR        = RADIO_INTERRUPTS;
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = RADIO_TASKS_DISABLE_TASKS_DISABLE_Trigger << RADIO_TASKS_DISABLE_TASKS_DISABLE_Pos;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}
    radio_vars.state = RADIO_STATE_IDLE;
}

int8_t db_radio_rssi(void) {
    return (uint8_t)NRF_RADIO->RSSISAMPLE * -1;
}


void db_radio_tx_prepare(const uint8_t *tx_buffer, uint8_t length) {
    radio_vars.pdu.length = length;
    memcpy(radio_vars.pdu.payload, tx_buffer, length);
}

void db_radio_tx_dispatch(void) {
    NRF_RADIO->SHORTS = RADIO_SHORTS_COMMON | (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    if (radio_vars.state == RADIO_STATE_IDLE) {

        // Enable the Radio to send the packet
        NRF_RADIO->EVENTS_DISABLED = 0;  // We must use EVENT_DISABLED, if we use EVENT_END. the interrupts will be enabled in the time between the END event and the Disable event, triggering an undesired interrupt
        NRF_RADIO->TASKS_TXEN      = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger << RADIO_TASKS_TXEN_TASKS_TXEN_Pos;
        // Wait for transmission to end and the radio to be disabled
        while (NRF_RADIO->EVENTS_DISABLED == 0) {}

        // We re-enable interrupts AFTER the packet is sent, to avoid triggering an EVENT_ADDRESS and EVENT_DISABLED interrupt with the outgoing packet
        // We also clear both flags to avoid insta-triggering an interrupt as soon as we assert INTENSET
        NRF_RADIO->EVENTS_ADDRESS  = 0;
        NRF_RADIO->EVENTS_DISABLED = 0;
        _radio_enable();
    }
    radio_vars.state = RADIO_STATE_RX;
}


//=========================== private ==========================================

static void _radio_enable(void) {
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->INTENSET        = RADIO_INTERRUPTS;
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

    if (NRF_RADIO->EVENTS_ADDRESS) {
        NRF_RADIO->EVENTS_ADDRESS = 0;
        radio_vars.state |= RADIO_STATE_BUSY;
    }

    if (NRF_RADIO->EVENTS_DISABLED) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (radio_vars.state == (RADIO_STATE_BUSY | RADIO_STATE_RX)) {
            if (NRF_RADIO->CRCSTATUS != RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
                puts("Invalid CRC");
            } else if (radio_vars.callback) {
                radio_vars.callback(radio_vars.pdu.payload, radio_vars.pdu.length);
            }
            radio_vars.state = RADIO_STATE_RX;
        } else {  // TX
            radio_vars.state = RADIO_STATE_RX;
        }
    }
}
