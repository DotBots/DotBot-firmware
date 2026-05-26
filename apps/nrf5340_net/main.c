/**
 * @file
 * @defgroup project_nrf5340_net_core   nRF5340 network core
 * @ingroup projects
 * @brief This application is used to control the radio and rng peripherals and to interact with the application core
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 */

#include <stdbool.h>
#include <string.h>
#include <nrf.h>
// Include BSP headers
#include "ipc.h"
#include "radio.h"
#include "rng.h"
// Include DRV headers
#include "tdma_client.h"
#include "tdma_server.h"

//=========================== defines ==========================================
typedef struct {
    bool      _data_received;
    ipc_req_t _req_received;
    // TDMA server variables
    uint32_t           frame_duration_us;
    uint16_t           num_clients;
    uint16_t           table_index;
    tdma_table_entry_t client;
} nrf53_net_vars_t;

//=========================== variables ========================================

volatile __attribute__((section(".shared_data"))) ipc_shared_data_t ipc_shared_data;

static nrf53_net_vars_t _nrf53_net_vars = { 0 };

//=========================== functions ========================================

void radio_callback(uint8_t *packet, uint8_t length) {
    mutex_lock();
    ipc_shared_data.radio.rx_pdu.length = length;
    memcpy((void *)ipc_shared_data.radio.rx_pdu.buffer, packet, length);
    mutex_unlock();
    _nrf53_net_vars._data_received = true;
}

void tdma_client_callback(uint8_t *packet, uint8_t length) {
    mutex_lock();
    ipc_shared_data.tdma_client.rx_pdu.length = length;
    memcpy((void *)ipc_shared_data.tdma_client.rx_pdu.buffer, packet, length);
    mutex_unlock();
    _nrf53_net_vars._data_received = true;
}

void tdma_server_callback(uint8_t *packet, uint8_t length) {
    mutex_lock();
    ipc_shared_data.tdma_server.rx_pdu.length = length;
    memcpy((void *)ipc_shared_data.tdma_server.rx_pdu.buffer, packet, length);
    mutex_unlock();
    _nrf53_net_vars._data_received = true;
}

//=========================== main =============================================

int main(void) {

    // Initialize local variables
    _nrf53_net_vars._data_received = false;
    _nrf53_net_vars._req_received  = DB_IPC_REQ_NONE;

    NRF_IPC_NS->INTENSET                       = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_NS->SEND_CNF[DB_IPC_CHAN_RADIO_RX] = 1 << DB_IPC_CHAN_RADIO_RX;
    NRF_IPC_NS->RECEIVE_CNF[DB_IPC_CHAN_REQ]   = 1 << DB_IPC_CHAN_REQ;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, 1);

    ipc_shared_data.net_ready = true;

    while (1) {
        __WFE();
        if (_nrf53_net_vars._data_received) {
            _nrf53_net_vars._data_received               = false;
            NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_RADIO_RX] = 1;
        }
        if (_nrf53_net_vars._req_received != DB_IPC_REQ_NONE) {
            ipc_shared_data.net_ack = false;
            switch (_nrf53_net_vars._req_received) {
                // RADIO functions
                case DB_IPC_RADIO_INIT_REQ:
                    db_radio_init(&radio_callback, ipc_shared_data.radio.mode);
                    break;
                case DB_IPC_RADIO_FREQ_REQ:
                    db_radio_set_frequency(ipc_shared_data.radio.frequency);
                    break;
                case DB_IPC_RADIO_CHAN_REQ:
                    db_radio_set_channel(ipc_shared_data.radio.channel);
                    break;
                case DB_IPC_RADIO_ADDR_REQ:
                    db_radio_set_network_address(ipc_shared_data.radio.addr);
                    break;
                case DB_IPC_RADIO_RX_REQ:
                    db_radio_rx();
                    break;
                case DB_IPC_RADIO_DIS_REQ:
                    db_radio_disable();
                    break;
                case DB_IPC_RADIO_TX_REQ:
                    db_radio_tx((uint8_t *)ipc_shared_data.radio.tx_pdu.buffer, ipc_shared_data.radio.tx_pdu.length);
                    break;
                case DB_IPC_RADIO_RSSI_REQ:
                    ipc_shared_data.radio.rssi = db_radio_rssi();
                    break;

                // RNG functions
                case DB_IPC_RNG_INIT_REQ:
                    db_rng_init();
                    break;
                case DB_IPC_RNG_READ_REQ:
                    db_rng_read((uint8_t *)&ipc_shared_data.rng.value);
                    break;

                // TDMA Client functions
                case DB_IPC_TDMA_CLIENT_INIT_REQ:
                    db_tdma_client_init(&tdma_client_callback, ipc_shared_data.tdma_client.mode, ipc_shared_data.tdma_client.frequency);
                    break;
                case DB_IPC_TDMA_CLIENT_SET_TABLE_REQ:
                    db_tdma_client_set_table((const tdma_client_table_t *)&ipc_shared_data.tdma_client.table_set);
                    break;
                case DB_IPC_TDMA_CLIENT_GET_TABLE_REQ:
                    db_tdma_client_get_table((tdma_client_table_t *)&ipc_shared_data.tdma_client.table_get);
                    break;
                case DB_IPC_TDMA_CLIENT_TX_REQ:
                    db_tdma_client_tx((uint8_t *)ipc_shared_data.tdma_client.tx_pdu.buffer, ipc_shared_data.tdma_client.tx_pdu.length);
                    break;
                case DB_IPC_TDMA_CLIENT_FLUSH_REQ:
                    db_tdma_client_flush();
                    break;
                case DB_IPC_TDMA_CLIENT_EMPTY_REQ:
                    db_tdma_client_empty();
                    break;
                case DB_IPC_TDMA_CLIENT_STATUS_REQ:
                    ipc_shared_data.tdma_client.registration_state = db_tdma_client_get_status();
                    break;

                // TDMA Server functions
                case DB_IPC_TDMA_SERVER_INIT_REQ:
                    db_tdma_server_init(&tdma_server_callback, ipc_shared_data.tdma_server.mode, ipc_shared_data.tdma_server.frequency);
                    break;
                case DB_IPC_TDMA_SERVER_GET_TABLE_REQ:
                    db_tdma_server_get_table_info(&_nrf53_net_vars.frame_duration_us, &_nrf53_net_vars.num_clients, &_nrf53_net_vars.table_index);
                    ipc_shared_data.tdma_server.frame_duration_us = _nrf53_net_vars.frame_duration_us;
                    ipc_shared_data.tdma_server.num_clients       = _nrf53_net_vars.num_clients;
                    ipc_shared_data.tdma_server.table_index       = _nrf53_net_vars.table_index;
                    break;
                case DB_IPC_TDMA_SERVER_GET_CLIENT_REQ:
                    // Copy the client info to an intermediate variable
                    db_tdma_server_get_client_info((tdma_table_entry_t *)&_nrf53_net_vars.client, ipc_shared_data.tdma_server.client_id);
                    // Copy the intermediate variable to the ipc shared variable
                    ipc_shared_data.tdma_server.client_entry.client      = _nrf53_net_vars.client.client;
                    ipc_shared_data.tdma_server.client_entry.rx_duration = _nrf53_net_vars.client.rx_duration;
                    ipc_shared_data.tdma_server.client_entry.rx_start    = _nrf53_net_vars.client.rx_start;
                    ipc_shared_data.tdma_server.client_entry.tx_duration = _nrf53_net_vars.client.tx_duration;
                    ipc_shared_data.tdma_server.client_entry.tx_start    = _nrf53_net_vars.client.tx_start;
                    break;
                case DB_IPC_TDMA_SERVER_TX_REQ:
                    db_tdma_server_tx((uint8_t *)ipc_shared_data.tdma_server.tx_pdu.buffer, ipc_shared_data.tdma_server.tx_pdu.length);
                    break;
                case DB_IPC_TDMA_SERVER_FLUSH_REQ:
                    db_tdma_server_flush();
                    break;
                case DB_IPC_TDMA_SERVER_EMPTY_REQ:
                    db_tdma_server_empty();
                    break;
                default:
                    break;
            }
            ipc_shared_data.net_ack       = true;
            _nrf53_net_vars._req_received = DB_IPC_REQ_NONE;
            ipc_shared_data.req           = DB_IPC_REQ_NONE;  // used in the app-core ipc.h to check that the request was properly fulfilled.
            __NOP();
        }
    };
}

void IPC_IRQHandler(void) {
    if (NRF_IPC_NS->EVENTS_RECEIVE[DB_IPC_CHAN_REQ]) {
        NRF_IPC_NS->EVENTS_RECEIVE[DB_IPC_CHAN_REQ] = 0;
        _nrf53_net_vars._req_received               = ipc_shared_data.req;
    }
}
