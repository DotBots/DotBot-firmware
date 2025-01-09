#ifndef __RADIO_H
#define __RADIO_H

/**
 * @defgroup    bsp_radio   Radio support
 * @ingroup     bsp
 * @brief       Control the radio peripheral
 *
 * This radio driver supports BLE 1Mbit, 2Mbit, Long Range 125Kbit, Long Range 500Kbit
 * and IEEE 802.15.4 250Kbit
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 *
 * @copyright Inria, 2022-2024
 * @}
 */

#include <stdint.h>
#include <nrf.h>

//=========================== defines ==========================================

#ifndef DEFAULT_NETWORK_ADDRESS
#define DEFAULT_NETWORK_ADDRESS 0x12345678UL  ///< Default network address
#endif

#define DB_BLE_PAYLOAD_MAX_LENGTH        UINT8_MAX
#define DB_IEEE802154_PAYLOAD_MAX_LENGTH (125UL)  ///< Total usable payload for IEEE 802.15.4 is 125 octets (PSDU) when CRC is activated

/// Modes supported by the radio
typedef enum {
    DB_RADIO_BLE_1MBit,
    DB_RADIO_BLE_2MBit,
    DB_RADIO_BLE_LR125Kbit,
    DB_RADIO_BLE_LR500Kbit,
    DB_RADIO_IEEE802154_250Kbit
} db_radio_mode_t;

typedef void (*radio_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

//=========================== public ===========================================

/**
 * @brief Initializes the RADIO peripheral
 *
 * After this function you must explicitly set the frequency of the radio
 * with the db_radio_set_frequency function.
 *
 * @param[in] callback pointer to a function that will be called each time a packet is received.
 * @param[in] mode     Mode used by the radio BLE (1MBit, 2MBit, LR125KBit, LR500Kbit) or IEEE 802.15.4 (250Kbit)
 *
 */
void db_radio_init(radio_cb_t callback, db_radio_mode_t mode);

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 *
 * Radio frequency 2400 + freq (MHz) [0, 100]
 *
 * @param[in] freq Frequency of the radio [0, 100]
 */
void db_radio_set_frequency(uint8_t freq);

/**
 * @brief Set the physical channel used of the radio
 *
 * BLE channels in the interval [0-39]
 * Channels 37, 38 and 39 are BLE advertising channels.
 *
 * IEEE 802.15.4 in the interval [11 - 26]
 * Channels range from 2405 MHz (channel 11) to 2480 MHz (channel 26)
 *
 * @param[in] channel   Channel used by the radio
 */
void db_radio_set_channel(uint8_t channel);

/**
 * @brief Set the network address used to send/receive radio packets
 *
 * @param[in] addr Network address
 */
void db_radio_set_network_address(uint32_t addr);

/**
 * @brief Sends a single packet through the Radio
 *
 * NOTE: Must configure the radio and the frequency before calling this function.
 * (with the functions db_radio_init db_radio_set_frequency).
 *
 * NOTE: The radio must not be receiving packets when calling this function.
 * (first call db_radio_disable if needed)
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void db_radio_tx(const uint8_t *packet, uint8_t length);

/**
 * @brief Starts Receiving packets through the Radio
 *
 * NOTE: Must configure the radio and the frequency before calling this function.
 * (with the functions db_radio_init db_radio_set_frequency).
 *
 */
void db_radio_rx(void);

/**
 * @brief Reads the RSSI of a received packet
 *
 * Should be called after a packet is received, e.g. in the radio callback
 */
int8_t db_radio_rssi(void);

/**
 * @brief Disables the radio, no packet can be received and energy consumption is minimal
 */
void db_radio_disable(void);

void db_radio_tx_prepare(const uint8_t *tx_buffer, uint8_t length);
void db_radio_tx_dispatch(void);

#endif
