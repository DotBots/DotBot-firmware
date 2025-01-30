#ifndef __IPC_H
#define __IPC_H

/**
 * @defgroup    bsp_ipc Inter-Processor Communication
 * @ingroup     bsp
 * @brief       Control the IPC peripheral (nRF53 only)
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <nrf.h>
#include <stdbool.h>
#include <stdint.h>
#include "radio.h"
#include "tdma_client.h"
#include "tdma_server.h"
#include "protocol.h"
#include "timer_hf.h"

#if defined(NRF_APPLICATION)
#define NRF_MUTEX NRF_MUTEX_NS
#elif defined(NRF_NETWORK)
#define NRF_MUTEX NRF_APPMUTEX_NS
#endif

#define IPC_IRQ_PRIORITY (1)

typedef enum {
    DB_IPC_REQ_NONE,                    ///< Sorry, but nothing
    DB_IPC_RADIO_INIT_REQ,              ///< Request for radio initialization
    DB_IPC_RADIO_FREQ_REQ,              ///< Request for radio set frequency
    DB_IPC_RADIO_CHAN_REQ,              ///< Request for radio set channel
    DB_IPC_RADIO_ADDR_REQ,              ///< Request for radio set network address
    DB_IPC_RADIO_RX_REQ,                ///< Request for radio rx
    DB_IPC_RADIO_DIS_REQ,               ///< Request for radio disable
    DB_IPC_RADIO_TX_REQ,                ///< Request for radio tx
    DB_IPC_RADIO_RSSI_REQ,              ///< Request for RSSI
    DB_IPC_RNG_INIT_REQ,                ///< Request for rng init
    DB_IPC_RNG_READ_REQ,                ///< Request for rng read
    DB_IPC_TDMA_CLIENT_INIT_REQ,        ///< Request for TDMA client initialization
    DB_IPC_TDMA_CLIENT_SET_TABLE_REQ,   ///< Request for setting the TDMA client timing table
    DB_IPC_TDMA_CLIENT_GET_TABLE_REQ,   ///< Request for reading the TDMA client timing table
    DB_IPC_TDMA_CLIENT_TX_REQ,          ///< Request for a TDMA client TX
    DB_IPC_TDMA_CLIENT_FLUSH_REQ,       ///< Request for flushing the TDMA client message buffer
    DB_IPC_TDMA_CLIENT_EMPTY_REQ,       ///< Request for erasing the TDMA client message buffer
    DB_IPC_TDMA_CLIENT_STATUS_REQ,      ///< Request for reading the TDMA client driver status
    DB_IPC_TDMA_SERVER_INIT_REQ,        ///< Request for TDMA server initialization
    DB_IPC_TDMA_SERVER_GET_TABLE_REQ,   ///< Request for reading the TDMA server timing table general info
    DB_IPC_TDMA_SERVER_GET_CLIENT_REQ,  ///< Request for reading the info about a specific client
    DB_IPC_TDMA_SERVER_TX_REQ,          ///< Request for a TDMA server TX
    DB_IPC_TDMA_SERVER_FLUSH_REQ,       ///< Request for flushing the TDMA server message buffer
    DB_IPC_TDMA_SERVER_EMPTY_REQ,       ///< Request for erasing the TDMA server message buffer
} ipc_req_t;

typedef enum {
    DB_IPC_CHAN_REQ      = 0,  ///< Channel used for request events
    DB_IPC_CHAN_RADIO_RX = 1,  ///< Channel used for radio RX events
} ipc_channels_t;

typedef struct __attribute__((packed)) {
    uint8_t length;             ///< Length of the pdu in bytes
    uint8_t buffer[UINT8_MAX];  ///< Buffer containing the pdu data
} ipc_radio_pdu_t;

typedef struct __attribute__((packed)) {
    db_radio_mode_t mode;       ///< db_radio_init function parameters
    uint8_t         frequency;  ///< db_set_frequency function parameters
    uint8_t         channel;    ///< db_set_channel function parameters
    uint32_t        addr;       ///< db_set_network_address function parameters
    ipc_radio_pdu_t tx_pdu;     ///< PDU to send
    ipc_radio_pdu_t rx_pdu;     ///< Received pdu
    int8_t          rssi;       ///< RSSI value
} ipc_radio_data_t;

typedef struct {
    uint8_t value;  ///< Byte containing the random value read
} ipc_rng_data_t;

typedef struct __attribute__((packed)) {
    db_radio_mode_t              mode;                ///< db_radio_init function parameters
    uint8_t                      frequency;           ///< db_set_frequency function parameters
    tdma_client_table_t          table_set;           ///< db_tdma_client_set_table function parameter
    tdma_client_table_t          table_get;           ///< db_tdma_client_get_table function parameter
    ipc_radio_pdu_t              tx_pdu;              ///< PDU to send
    ipc_radio_pdu_t              rx_pdu;              ///< Received pdu
    db_tdma_registration_state_t registration_state;  ///< db_tdma_client_get_status return value
} ipc_tdma_client_data_t;

typedef struct __attribute__((packed)) {
    db_radio_mode_t    mode;               ///< db_radio_init function parameters
    uint8_t            frequency;          ///< db_set_frequency function parameters
    uint32_t           frame_duration_us;  ///< db_tdma_server_get_table_info function parameter
    uint16_t           num_clients;        ///< db_tdma_server_get_table_info function parameter
    uint16_t           table_index;        ///< db_tdma_server_get_table_info function parameter
    uint8_t            client_id;          ///< db_tdma_server_get_client_info function parameter
    tdma_table_entry_t client_entry;       ///< db_tdma_server_get_client_info function parameter
    ipc_radio_pdu_t    tx_pdu;             ///< PDU to send
    ipc_radio_pdu_t    rx_pdu;             ///< Received pdu
} ipc_tdma_server_data_t;

typedef struct __attribute__((packed)) {
    bool                   net_ready;    ///< Network core is ready
    bool                   net_ack;      ///< Network core acked the latest request
    ipc_req_t              req;          ///< IPC network request
    ipc_radio_data_t       radio;        ///< Radio shared data
    ipc_rng_data_t         rng;          ///< Rng share data
    ipc_tdma_client_data_t tdma_client;  ///< TDMA client drv shared data
    ipc_tdma_server_data_t tdma_server;  ///< TDMA server drv shared data
} ipc_shared_data_t;

/**
 * @brief Lock the mutex, blocks until the mutex is locked
 */
static inline void mutex_lock(void) {
    while (NRF_MUTEX->MUTEX[0]) {}
}

/**
 * @brief Unlock the mutex, has no effect if the mutex is already unlocked
 */
static inline void mutex_unlock(void) {
    NRF_MUTEX->MUTEX[0] = 0;
}

void db_ipc_network_call(ipc_req_t req);

void release_network_core(void);

#endif
