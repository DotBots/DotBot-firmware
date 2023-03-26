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

//=========================== variables =========================================

static bool _data_received = false;

//=========================== functions =========================================

void radio_callback(uint8_t *packet, uint8_t length) {
    mutex_lock();
    ipc_shared_data.radio.rx_param.length = length;
    memcpy((void *)ipc_shared_data.radio.rx_param.buffer, packet, length);
    mutex_unlock();
    _data_received = true;
}

//=========================== main ==============================================

int main(void) {
    NRF_IPC_NS->INTENSET       = IPC_INTENSET_RECEIVE1_Enabled << IPC_INTENSET_RECEIVE1_Pos;
    NRF_IPC_NS->SEND_CNF[0]    = IPC_SEND_CNF_CHEN0_Enable << IPC_SEND_CNF_CHEN0_Pos;
    NRF_IPC_NS->RECEIVE_CNF[1] = IPC_SEND_CNF_CHEN1_Enable << IPC_SEND_CNF_CHEN1_Pos;

    NVIC_EnableIRQ(IPC_IRQn);

    mutex_lock();
    ipc_shared_data.event     = DB_IPC_NET_READY_REQ;
    NRF_IPC_NS->TASKS_SEND[0] = 1;
    mutex_unlock();

    while (1) {
        __WFE();
        if (_data_received) {
            mutex_lock();
            ipc_shared_data.event     = DB_IPC_RADIO_RX_REQ;
            NRF_IPC_NS->TASKS_SEND[0] = 1;
            mutex_unlock();
            _data_received = false;
        }
        switch (ipc_shared_data.event) {
            case DB_IPC_RADIO_INIT_REQ:
                mutex_lock();
                db_radio_init(&radio_callback, ipc_shared_data.radio.init_param.mode);
                ipc_shared_data.event     = DB_IPC_RADIO_INIT_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_FREQ_REQ:
                mutex_lock();
                db_radio_set_frequency(ipc_shared_data.radio.freq_param.frequency);
                ipc_shared_data.event     = DB_IPC_RADIO_FREQ_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_CHAN_REQ:
                mutex_lock();
                db_radio_set_channel(ipc_shared_data.radio.chan_param.channel);
                ipc_shared_data.event     = DB_IPC_RADIO_CHAN_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_ADDR_REQ:
                mutex_lock();
                db_radio_set_network_address(ipc_shared_data.radio.addr_param.addr);
                ipc_shared_data.event     = DB_IPC_RADIO_ADDR_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_RX_EN_REQ:
                mutex_lock();
                db_radio_rx_enable();
                ipc_shared_data.event     = DB_IPC_RADIO_RX_EN_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_RX_DIS_REQ:
                mutex_lock();
                db_radio_rx_disable();
                ipc_shared_data.event     = DB_IPC_RADIO_RX_DIS_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RADIO_TX_REQ:
                mutex_lock();
                db_radio_tx((uint8_t *)ipc_shared_data.radio.tx_param.buffer, ipc_shared_data.radio.tx_param.length);
                ipc_shared_data.event     = DB_IPC_RADIO_TX_ACK;
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                mutex_unlock();
                break;
            case DB_IPC_RNG_INIT_REQ:
                mutex_lock();
                db_rng_init();
                ipc_shared_data.event = DB_IPC_RNG_INIT_ACK;
                mutex_unlock();
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                break;
            case DB_IPC_RNG_READ_REQ:
                mutex_lock();
                db_rng_read((uint8_t *)&ipc_shared_data.rng.value);
                ipc_shared_data.event = DB_IPC_RNG_READ_ACK;
                mutex_unlock();
                NRF_IPC_NS->TASKS_SEND[0] = 1;
                break;
            default:
                break;
        }
    };
}

void IPC_IRQHandler(void) {
    if (NRF_IPC_NS->EVENTS_RECEIVE[1]) {
        NRF_IPC_NS->EVENTS_RECEIVE[1] = 0;
    }

    NVIC_ClearPendingIRQ(IPC_IRQn);
}
