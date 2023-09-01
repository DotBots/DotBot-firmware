/**
 * @file radio_nrf5340_app.c
 * @addtogroup BSP
 *
 * @brief  nrf5340-app-specific definition of the "radio" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "clock.h"
#include "ipc.h"
#include "radio.h"
#include "timer_hf.h"

//=========================== variables ========================================

static radio_cb_t _radio_callback = NULL;

static bool _ack_received[] = {
    [DB_IPC_NET_READY_ACK]    = false,
    [DB_IPC_RADIO_INIT_ACK]   = false,
    [DB_IPC_RADIO_FREQ_ACK]   = false,
    [DB_IPC_RADIO_CHAN_ACK]   = false,
    [DB_IPC_RADIO_ADDR_ACK]   = false,
    [DB_IPC_RADIO_RX_EN_ACK]  = false,
    [DB_IPC_RADIO_RX_DIS_ACK] = false,
    [DB_IPC_RADIO_TX_ACK]     = false,
    [DB_IPC_RNG_INIT_ACK]     = false,
    [DB_IPC_RNG_READ_ACK]     = false,
};

//========================== functions =========================================

static inline void _network_call(ipc_event_type_t req, ipc_event_type_t ack) {
    if (req != DB_IPC_NONE) {
        ipc_shared_data.event                  = req;
        NRF_IPC_S->TASKS_SEND[DB_IPC_CHAN_REQ] = 1;
        mutex_unlock();
    }
    while (!_ack_received[ack]) {}
    _ack_received[ack] = false;
}

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback, db_radio_ble_mode_t mode) {
    // On nrf53 configure constant latency mode for better performances
    NRF_POWER_S->TASKS_CONSTLAT = 1;

    db_hfclk_init();

    // VREQCTRL (address at 0x41004000 => periph ID is 4)
    NRF_SPU_S->PERIPHID[4].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // POWER (address at 0x41005000 => periph ID is 5)
    NRF_SPU_S->PERIPHID[5].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // RADIO (address at 0x41008000 => periph ID is 8)
    NRF_SPU_S->PERIPHID[8].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos |
                                   SPU_PERIPHID_PERM_DMASEC_NonSecure << SPU_PERIPHID_PERM_DMASEC_Pos);

    // IPC (address at 0x41012000 => periph ID is 18)
    NRF_SPU_S->PERIPHID[18].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // APPMUTEX (address at 0x41030000 => periph ID is 48)
    NRF_SPU_S->PERIPHID[48].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // Define RAMREGION 2 (0x20004000 to 0x20005FFF, e.g 8KiB) as non secure. It's used to share data between cores
    NRF_SPU_S->RAMREGION[2].PERM = (SPU_RAMREGION_PERM_READ_Enable << SPU_RAMREGION_PERM_READ_Pos |
                                    SPU_RAMREGION_PERM_WRITE_Enable << SPU_RAMREGION_PERM_WRITE_Pos |
                                    SPU_RAMREGION_PERM_SECATTR_Non_Secure << SPU_RAMREGION_PERM_SECATTR_Pos);

    NRF_IPC_S->INTENSET                          = (1 << DB_IPC_CHAN_ACK | 1 << DB_IPC_CHAN_RADIO_RX);
    NRF_IPC_S->SEND_CNF[DB_IPC_CHAN_REQ]         = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_S->RECEIVE_CNF[DB_IPC_CHAN_ACK]      = 1 << DB_IPC_CHAN_ACK;
    NRF_IPC_S->RECEIVE_CNF[DB_IPC_CHAN_RADIO_RX] = 1 << DB_IPC_CHAN_RADIO_RX;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);

    // Start the network core
    if (NRF_RESET_NS->NETWORK.FORCEOFF != 0) {
        NRF_RESET_NS->NETWORK.FORCEOFF = 0;
        _network_call(DB_IPC_NONE, DB_IPC_NET_READY_ACK);
    }

    if (callback) {
        _radio_callback = callback;
    }

    mutex_lock();
    ipc_shared_data.radio.mode = mode;
    _network_call(DB_IPC_RADIO_INIT_REQ, DB_IPC_RADIO_INIT_ACK);
}

void db_radio_set_frequency(uint8_t freq) {
    mutex_lock();
    ipc_shared_data.radio.frequency = freq;
    _network_call(DB_IPC_RADIO_FREQ_REQ, DB_IPC_RADIO_FREQ_ACK);
}

void db_radio_set_channel(uint8_t channel) {
    mutex_lock();
    ipc_shared_data.radio.channel = channel;
    _network_call(DB_IPC_RADIO_CHAN_REQ, DB_IPC_RADIO_CHAN_ACK);
}

void db_radio_set_network_address(uint32_t addr) {
    mutex_lock();
    ipc_shared_data.radio.addr = addr;
    _network_call(DB_IPC_RADIO_ADDR_REQ, DB_IPC_RADIO_ADDR_ACK);
}

void db_radio_tx(uint8_t *tx_buffer, uint8_t length) {
    mutex_lock();
    ipc_shared_data.radio.tx_pdu.length = length;
    memcpy((void *)ipc_shared_data.radio.tx_pdu.buffer, tx_buffer, length);
    _network_call(DB_IPC_RADIO_TX_REQ, DB_IPC_RADIO_TX_ACK);
}

void db_radio_rx(void) {
    mutex_lock();
    _network_call(DB_IPC_RADIO_RX_EN_REQ, DB_IPC_RADIO_RX_EN_ACK);
}

int8_t db_radio_rssi(void) {
    mutex_lock();
    _network_call(DB_IPC_RADIO_RSSI_REQ, DB_IPC_RADIO_RSSI_ACK);
    return ipc_shared_data.radio.rssi;
}

void db_radio_disable(void) {
    mutex_lock();
    _network_call(DB_IPC_RADIO_RX_DIS_REQ, DB_IPC_RADIO_RX_DIS_ACK);
}

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_ACK]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_ACK] = 0;
        mutex_lock();
        _ack_received[ipc_shared_data.event] = true;
        mutex_unlock();
    }
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX] = 0;
        if (_radio_callback) {
            mutex_lock();
            _radio_callback((uint8_t *)ipc_shared_data.radio.rx_pdu.buffer, ipc_shared_data.radio.rx_pdu.length);
            mutex_unlock();
        }
    }
}
