#ifndef __IPC_H
#define __IPC_H

/**
 * @file ipc.h
 * @addtogroup BSP
 *
 * @brief  Declaration for "ipc" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <nrf.h>
#include <stdbool.h>
#include <stdint.h>
#include "radio.h"
#include "timer_hf.h"

#if defined(NRF_APPLICATION)
#define NRF_MUTEX NRF_MUTEX_NS
#elif defined(NRF_NETWORK)
#define NRF_MUTEX NRF_APPMUTEX_NS
#endif

#define IPC_IRQ_PRIORITY (1)

typedef enum {
    DB_IPC_NONE,            ///< Sorry, but nothing
    DB_IPC_NET_READY_ACK,   ///< Network core is ready
    DB_IPC_RADIO_INIT_REQ,  ///< Request for radio initialization
    DB_IPC_RADIO_INIT_ACK,  ///< Acknowledment for radio initialization
    DB_IPC_RADIO_FREQ_REQ,  ///< Request for radio set frequency
    DB_IPC_RADIO_FREQ_ACK,  ///< Acknowledment for radio set frequency
    DB_IPC_RADIO_CHAN_REQ,  ///< Request for radio set channel
    DB_IPC_RADIO_CHAN_ACK,  ///< Acknowledment for radio set channel
    DB_IPC_RADIO_ADDR_REQ,  ///< Request for radio set network address
    DB_IPC_RADIO_ADDR_ACK,  ///< Acknowledment for radio set network address
    DB_IPC_RADIO_RX_REQ,    ///< Request for radio rx
    DB_IPC_RADIO_RX_ACK,    ///< Acknowledment for radio rx
    DB_IPC_RADIO_DIS_REQ,   ///< Request for radio disable
    DB_IPC_RADIO_DIS_ACK,   ///< Acknowledment for radio disable
    DB_IPC_RADIO_TX_REQ,    ///< Request for radio tx
    DB_IPC_RADIO_TX_ACK,    ///< Acknowledment for radio tx
    DB_IPC_RADIO_RSSI_REQ,  ///< Request for RSSI
    DB_IPC_RADIO_RSSI_ACK,  ///< Acknowledment for RSSI
    DB_IPC_RNG_INIT_REQ,    ///< Request for rng init
    DB_IPC_RNG_INIT_ACK,    ///< Acknowledment for rng init
    DB_IPC_RNG_READ_REQ,    ///< Request for rng read
    DB_IPC_RNG_READ_ACK,    ///< Acknowledment for rng read
} ipc_event_type_t;

typedef enum {
    DB_IPC_CHAN_REQ      = 0,  ///< Channel used for request events
    DB_IPC_CHAN_ACK      = 1,  ///< Channel used for acknownlegment events
    DB_IPC_CHAN_RADIO_RX = 2,  ///< Channel used for radio RX events
} ipc_channels_t;

typedef struct __attribute__((packed)) {
    uint8_t length;             ///< Length of the pdu in bytes
    uint8_t buffer[UINT8_MAX];  ///< Buffer containing the pdu data
} ipc_radio_pdu_t;

typedef struct __attribute__((packed)) {
    db_radio_ble_mode_t mode;       ///< db_radio_init function parameters
    uint8_t             frequency;  ///< db_set_frequency function parameters
    uint8_t             channel;    ///< db_set_channel function parameters
    uint32_t            addr;       ///< db_set_network_address function parameters
    ipc_radio_pdu_t     tx_pdu;     ///< PDU to send
    ipc_radio_pdu_t     rx_pdu;     ///< Received pdu
    int8_t              rssi;       ///< RSSI value
} ipc_radio_data_t;

typedef struct {
    uint8_t value;  ///< Byte containing the random value read
} ipc_rng_data_t;

typedef struct __attribute__((packed)) {
    ipc_event_type_t event;  ///< IPC event
    ipc_radio_data_t radio;  ///< Radio shared data
    ipc_rng_data_t   rng;    ///< Rng share data
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

/**
 * @brief Variable in RAM containing the shared data structure
 */
volatile __attribute__((section(".ARM.__at_0x20004000"))) ipc_shared_data_t ipc_shared_data;

#endif
