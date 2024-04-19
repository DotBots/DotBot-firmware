#ifndef __RADIO_H_IEEE_802154
#define __RADIO_H_IEEE_802154

/**
 * @defgroup    bsp_radio   Radio support
 * @ingroup     bsp
 * @brief       Control the radio peripheral
 *
 * This radio driver supports IEEE 802.15.4.
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author Simoes Raphael <raphael.simoes@inria.fr>
 * @copyright Inria, 2022-2023
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

//=========================== defines ==========================================

#ifndef DEFAULT_NETWORK_ADDRESS
#define DEFAULT_NETWORK_ADDRESS 0x12345678UL  ///< Default network address
#endif

typedef void (*radio_ieee_802154_cb_t)(uint8_t *packet, uint8_t length, bool crc);  ///< Function pointer to the callback function called on packet receive

//=========================== public ===========================================

/**
 * @brief Initializes the RADIO peripheral
 *
 * After this function you must explicitly set the frequency of the radio
 * with the db_radio_set_frequency function.
 *
 * @param[in] callback pointer to a function that will be called each time a packet is received.
 *
 */
void db_radio_ieee_802154_init(radio_ieee_802154_cb_t callback);

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 *
 * Radio frequency 2400 + freq (MHz) [0, 100]
 *
 * @param[in] freq Frequency of the radio [0, 100]
 */
void db_radio_ieee_802154_set_frequency(uint8_t freq);

/**
 * @brief Set the physical channel used of the radio
 *
 * Channels 37, 38 and 39 are BLE advertising channels.
 *
 * @param[in] channel BLE channel used by the radio [0-39]
 */
void db_radio_ieee_802154_set_channel(uint8_t channel);

/**
 * @brief Set the TX power of the radio
 *
 * @param[in] power of the radio
 */
void db_radio_ieee_802154_set_tx_power(uint8_t power);

/**
 * @brief Set the network address used to send/receive radio packets
 *
 * @param[in] addr Network address
 */
void db_radio_ieee_802154_set_network_address(uint32_t addr);

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
void db_radio_ieee_802154_tx(const uint8_t *packet, uint8_t length);

/**
 * @brief Starts Receiving packets through the Radio
 *
 * NOTE: Must configure the radio and the frequency before calling this function.
 * (with the functions db_radio_init db_radio_set_frequency).
 *
 */
void db_radio_ieee_802154_rx(void);

/**
 * @brief Block Radio into TX idle state
 */
void db_radio_ieee_802154_tx_start(void);

/**
 * @brief Reads the RSSI of a received packet
 *
 * Should be called after a packet is received, e.g. in the radio callback
 */
int8_t db_radio_ieee_802154_rssi(void);

/**
 * @brief Disables the radio, no packet can be received and energy consumption is minimal
 */
void db_radio_ieee_802154_disable(void);

#endif
