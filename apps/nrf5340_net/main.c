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

//=========================== defines ==========================================
typedef struct {
    bool      _data_received;
    ipc_req_t _req_received;
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
