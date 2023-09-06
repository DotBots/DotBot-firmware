/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control the radio and rng peripherals and to interact with the application core
 *
 * @copyright Inria, 2023
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <nrf.h>
// Include BSP headers
#include "ipc.h"
#include "radio.h"
#include "rng.h"
#include "gpio.h"

//=========================== variables =========================================

static bool             _data_received  = false;
static ipc_event_type_t _event_received = DB_IPC_NONE;

//=========================== functions =========================================

void radio_callback(uint8_t *packet, uint8_t length) {
    mutex_lock();
    ipc_shared_data.radio.rx_pdu.length = length;
    memcpy((void *)ipc_shared_data.radio.rx_pdu.buffer, packet, length);
    _data_received = true;
}

//=========================== main ==============================================

int main(void) {

    // Configure constant latency mode for better performances
    NRF_POWER_NS->TASKS_CONSTLAT = 1;

    NRF_IPC_NS->INTENSET                       = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_NS->SEND_CNF[DB_IPC_CHAN_ACK]      = 1 << DB_IPC_CHAN_ACK;
    NRF_IPC_NS->SEND_CNF[DB_IPC_CHAN_RADIO_RX] = 1 << DB_IPC_CHAN_RADIO_RX;
    NRF_IPC_NS->RECEIVE_CNF[DB_IPC_CHAN_REQ]   = 1 << DB_IPC_CHAN_REQ;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, 1);

    mutex_lock();
    ipc_shared_data.event                   = DB_IPC_NET_READY_ACK;
    NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
    mutex_unlock();

    while (1) {
        __WFE();
        if (_data_received) {
            _data_received                               = false;
            NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_RADIO_RX] = 1;
            mutex_unlock();
        }
        switch (_event_received) {
            case DB_IPC_RADIO_INIT_REQ:
                mutex_lock();
                db_radio_init(&radio_callback, ipc_shared_data.radio.mode);
                ipc_shared_data.event                   = DB_IPC_RADIO_INIT_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_FREQ_REQ:
                mutex_lock();
                db_radio_set_frequency(ipc_shared_data.radio.frequency);
                ipc_shared_data.event                   = DB_IPC_RADIO_FREQ_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_CHAN_REQ:
                mutex_lock();
                db_radio_set_channel(ipc_shared_data.radio.channel);
                ipc_shared_data.event                   = DB_IPC_RADIO_CHAN_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_ADDR_REQ:
                mutex_lock();
                db_radio_set_network_address(ipc_shared_data.radio.addr);
                ipc_shared_data.event                   = DB_IPC_RADIO_ADDR_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_RX_REQ:
                mutex_lock();
                db_radio_rx();
                ipc_shared_data.event                   = DB_IPC_RADIO_RX_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_DIS_REQ:
                mutex_lock();
                db_radio_disable();
                ipc_shared_data.event                   = DB_IPC_RADIO_DIS_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_TX_REQ:
                mutex_lock();
                db_radio_tx((uint8_t *)ipc_shared_data.radio.tx_pdu.buffer, ipc_shared_data.radio.tx_pdu.length);
                ipc_shared_data.event                   = DB_IPC_RADIO_TX_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_RSSI_REQ:
                mutex_lock();
                ipc_shared_data.radio.rssi              = db_radio_rssi();
                ipc_shared_data.event                   = DB_IPC_RADIO_RSSI_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RNG_INIT_REQ:
                mutex_lock();
                db_rng_init();
                ipc_shared_data.event                   = DB_IPC_RNG_INIT_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RNG_READ_REQ:
                mutex_lock();
                db_rng_read((uint8_t *)&ipc_shared_data.rng.value);
                ipc_shared_data.event                   = DB_IPC_RNG_READ_ACK;
                NRF_IPC_NS->TASKS_SEND[DB_IPC_CHAN_ACK] = 1;
                mutex_unlock();
                break;
            default:
                break;
        }
        _event_received = DB_IPC_NONE;
    };
}

void IPC_IRQHandler(void) {
    if (NRF_IPC_NS->EVENTS_RECEIVE[DB_IPC_CHAN_REQ]) {
        NRF_IPC_NS->EVENTS_RECEIVE[DB_IPC_CHAN_REQ] = 0;
        _event_received                             = ipc_shared_data.event;
    }
}
