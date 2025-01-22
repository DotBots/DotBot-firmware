#ifndef __TDMA_SERVER_H
#define __TDMA_SERVER_H

/**
 * @defgroup    drv_tdma_server      TDMA server radio driver
 * @ingroup     drv
 * @brief       Driver for Time-Division-Multiple-Access for the Gateway radio
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2024-now
 * @}
 */

#include <stdint.h>
#include <nrf.h>
#include "gpio.h"
#include "radio.h"
#include "protocol.h"

//=========================== defines ==========================================

#define TDMA_SERVER_MAX_CLIENTS             100    ///< Max number of clients that can register with this server
#define TDMA_SERVER_TIME_SLOT_DURATION_US   2500   ///< default timeslot for a tdma slot in microseconds
#define TDMA_SERVER_MAX_GATEWAY_TX_DELAY_US 20000  ///< Max amount of microseconds that can elapse between gateway transmissions
/// Total amount of slots available in the tdma table, adds extra slots to MAX_CLIENTS to accomodate the gateway slots
#define TDMA_SERVER_MAX_TABLE_SLOTS \
    TDMA_SERVER_MAX_CLIENTS +       \
        TDMA_SERVER_MAX_CLIENTS / (TDMA_SERVER_MAX_GATEWAY_TX_DELAY / TDMA_SERVER_TIME_SLOT_DURATION - 1) + 1

//=========================== variables ========================================

/// Data type to store the info about a single TDMA Client
typedef struct {
    uint64_t client;       ///< ID of the client registered to in this time slot
    uint32_t rx_start;     ///< Time between the start of the frame and when the gateway starts transmitting
    uint32_t rx_duration;  ///< Duration the gateway will transmit messages
    uint32_t tx_start;     ///< Time between the start of the frame and the start of the DotBot's alloted frame
    uint32_t tx_duration;  ///< Duration of the DotBot's alloted frame.
} tdma_table_entry_t;

/// Data type to store the TDMA table
typedef struct {
    uint32_t           frame_duration_us;               ///< Duration of the entire TDMA frame [microseconds]
    uint16_t           num_clients;                     ///< Number of clients currently connected to the tdma server
    uint16_t           table_index;                     ///< index of the last entry in the tdma table, includes slots taken by the gateway
    tdma_table_entry_t table[TDMA_SERVER_MAX_CLIENTS];  ///< array of tdma clients
} tdma_server_table_t;

typedef void (*tdma_server_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

//=========================== prototypes ==========================================

/**
 * @brief Initializes the TDMA scheme
 *
 * Starts advertising registration packets, set a default tdma table
 * and inits the radio
 *
 * @param[in] callback             pointer to a function that will be called each time a packet is received.
 * @param[in] radio_mode           BLE mode used by the radio (1MBit, 2MBit, LR125KBit, LR500Kbit)
 * @param[in] radio_freq           Frequency of the radio [0, 100]
 *
 */
void db_tdma_server_init(tdma_server_cb_t callback, db_radio_mode_t radio_mode, uint8_t radio_freq);

/**
 * @brief Get general info about the current TDMA table
 *
 * @param[out] frame_duration_us  pointer to the frame duration
 * @param[out] num_clients        pointer to the number of clients
 * @param[out] table_index        pointer to the last index of the TDMA table
 */
void db_tdma_server_get_table_info(uint32_t *frame_duration_us, uint16_t *num_clients, uint16_t *table_index);

/**
 * @brief Get information about one client
 *
 * @param[in]   client_id  first client_id to copy
 * @param[out]  client     copy of the table entry of a single client
 */
void db_tdma_server_get_client_info(tdma_table_entry_t *client, uint8_t client_id);

/**
 * @brief Queues a single packet to send through the Radio
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void db_tdma_server_tx(const uint8_t *packet, uint8_t length);

/**
 * @brief Ignore TDMA table and send all pending packets immediatly
 *
 */
void db_tdma_server_flush(void);

/**
 * @brief Erase all pending packets in the TDMA queue
 *
 */
void db_tdma_server_empty(void);

#endif
