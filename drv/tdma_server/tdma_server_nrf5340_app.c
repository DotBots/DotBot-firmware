/**
 * @file
 * @ingroup drv_tdma_server
 *
 * @brief  nrf5340-app-specific implementation of the "tdma_server" drv module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ipc.h"
#include "radio.h"
#include "clock.h"
#include "tz.h"
#include "timer.h"
#include "device.h"

//=========================== variables ========================================

static tdma_server_cb_t _tdma_server_callback = NULL;

extern volatile __attribute__((section(".shared_data"))) ipc_shared_data_t ipc_shared_data;

//=========================== public ===========================================

void db_tdma_server_init(tdma_server_cb_t callback, db_radio_mode_t radio_mode, uint8_t radio_freq) {

    // APPMUTEX (address at 0x41030000 => periph ID is 48)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_APPMUTEX);

    // Define RAMREGION 2 (0x20004000 to 0x20005FFF, e.g 8KiB) as non secure. It's used to share data between cores
    db_configure_ram_non_secure(2, 1);

    NRF_IPC_S->INTENSET                          = 1 << DB_IPC_CHAN_RADIO_RX;
    NRF_IPC_S->SEND_CNF[DB_IPC_CHAN_REQ]         = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_S->RECEIVE_CNF[DB_IPC_CHAN_RADIO_RX] = 1 << DB_IPC_CHAN_RADIO_RX;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);

    // Start the network core
    release_network_core();

    // Store callback in a local variable
    if (callback) {
        _tdma_server_callback = callback;
    }

    // Store information in the shared data before sending it to the net-core
    ipc_shared_data.tdma_server.mode      = radio_mode;
    ipc_shared_data.tdma_server.frequency = radio_freq;

    // Initialize TDMA client drv in the net-core
    db_ipc_network_call(DB_IPC_TDMA_SERVER_INIT_REQ);
}

void db_tdma_server_get_table_info(uint32_t *frame_duration_us, uint16_t *num_clients, uint16_t *table_index) {

    // Request the network core to copy the table's data.
    db_ipc_network_call(DB_IPC_TDMA_SERVER_GET_TABLE_REQ);

    // Copy the variables over
    *frame_duration_us = ipc_shared_data.tdma_server.frame_duration_us;
    *num_clients       = ipc_shared_data.tdma_server.num_clients;
    *table_index       = ipc_shared_data.tdma_server.table_index;
}

void db_tdma_server_get_client_info(tdma_table_entry_t *client, uint8_t client_id) {

    // Request a specific client
    ipc_shared_data.tdma_server.client_id = client_id;

    // Request the network core to copy the table's data.
    db_ipc_network_call(DB_IPC_TDMA_SERVER_GET_CLIENT_REQ);

    // Copy and return one entry of the TDMA entry (one by one because memcpy doesnt like that ipc_shared_data is volatile)
    client->client      = ipc_shared_data.tdma_server.client_entry.client;
    client->rx_duration = ipc_shared_data.tdma_server.client_entry.rx_duration;
    client->rx_start    = ipc_shared_data.tdma_server.client_entry.rx_start;
    client->tx_duration = ipc_shared_data.tdma_server.client_entry.tx_duration;
    client->tx_start    = ipc_shared_data.tdma_server.client_entry.tx_start;
}

void db_tdma_server_tx(const uint8_t *packet, uint8_t length) {
    ipc_shared_data.tdma_server.tx_pdu.length = length;
    memcpy((void *)ipc_shared_data.tdma_server.tx_pdu.buffer, packet, length);
    db_ipc_network_call(DB_IPC_TDMA_SERVER_TX_REQ);
}

void db_tdma_server_flush(void) {
    db_ipc_network_call(DB_IPC_TDMA_SERVER_FLUSH_REQ);
}

void db_tdma_server_empty(void) {
    db_ipc_network_call(DB_IPC_TDMA_SERVER_EMPTY_REQ);
}

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX] = 0;
        if (_tdma_server_callback) {
            mutex_lock();
            _tdma_server_callback((uint8_t *)ipc_shared_data.tdma_server.rx_pdu.buffer, ipc_shared_data.tdma_server.rx_pdu.length);
            mutex_unlock();
        }
    }
}
