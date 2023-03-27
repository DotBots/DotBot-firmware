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

//========================== functions =========================================

static void _network_call(ipc_event_type_t req, ipc_event_type_t ack) {
    ipc_shared_data.event    = req;
    NRF_IPC_S->TASKS_SEND[1] = 1;
    mutex_unlock();
    while (ipc_shared_data.event != ack) {}
}

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback, db_radio_ble_mode_t mode) {
    db_hfclk_init();

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

    NRF_IPC_S->INTENSET       = IPC_INTENSET_RECEIVE0_Enabled << IPC_INTENSET_RECEIVE0_Pos;
    NRF_IPC_S->SEND_CNF[1]    = IPC_SEND_CNF_CHEN1_Enable << IPC_SEND_CNF_CHEN1_Pos;
    NRF_IPC_S->RECEIVE_CNF[0] = IPC_SEND_CNF_CHEN0_Enable << IPC_SEND_CNF_CHEN0_Pos;

    NVIC_EnableIRQ(IPC_IRQn);

    // Start the network core
    if (NRF_RESET_S->NETWORK.FORCEOFF != 0) {
        NRF_RESET_S->NETWORK.FORCEOFF = 0;

        while (ipc_shared_data.event != DB_IPC_NET_READY_REQ) {}
    }

    if (callback) {
        _radio_callback = callback;
    }

    mutex_lock();
    ipc_shared_data.radio.init_param.mode = mode;
    _network_call(DB_IPC_RADIO_INIT_REQ, DB_IPC_RADIO_INIT_ACK);
}

void db_radio_set_frequency(uint8_t freq) {
    mutex_lock();
    ipc_shared_data.radio.freq_param.frequency = freq;
    _network_call(DB_IPC_RADIO_FREQ_REQ, DB_IPC_RADIO_FREQ_ACK);
}

void db_radio_set_channel(uint8_t channel) {
    mutex_lock();
    ipc_shared_data.radio.chan_param.channel = channel;
    _network_call(DB_IPC_RADIO_CHAN_REQ, DB_IPC_RADIO_CHAN_ACK);
}

void db_radio_set_network_address(uint32_t addr) {
    mutex_lock();
    ipc_shared_data.radio.addr_param.addr = addr;
    _network_call(DB_IPC_RADIO_ADDR_REQ, DB_IPC_RADIO_ADDR_ACK);
}

void db_radio_tx(uint8_t *tx_buffer, uint8_t length) {
    mutex_lock();
    ipc_shared_data.radio.tx_param.length = length;
    memcpy((void *)ipc_shared_data.radio.tx_param.buffer, tx_buffer, length);
    _network_call(DB_IPC_RADIO_TX_REQ, DB_IPC_RADIO_TX_ACK);
}

void db_radio_rx_enable(void) {
    mutex_lock();
    _network_call(DB_IPC_RADIO_RX_EN_REQ, DB_IPC_RADIO_RX_EN_ACK);
}

void db_radio_rx_disable(void) {
    mutex_lock();
    _network_call(DB_IPC_RADIO_RX_DIS_REQ, DB_IPC_RADIO_RX_DIS_ACK);
}

//=========================== private ==========================================

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[0]) {
        NRF_IPC_S->EVENTS_RECEIVE[0] = 0;
        if (ipc_shared_data.event == DB_IPC_RADIO_RX_REQ && _radio_callback != NULL) {
            _radio_callback((uint8_t *)ipc_shared_data.radio.rx_param.buffer, ipc_shared_data.radio.rx_param.length);
        }
    }
}
