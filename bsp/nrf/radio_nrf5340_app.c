/**
 * @file
 * @ingroup bsp_radio
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

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback, db_radio_ble_mode_t mode) {
    db_hfclk_init();

    // Disable all DCDC regulators (use LDO)
    NRF_REGULATORS_S->VREGRADIO.DCDCEN = (REGULATORS_VREGRADIO_DCDCEN_DCDCEN_Disabled << REGULATORS_VREGRADIO_DCDCEN_DCDCEN_Pos);
    NRF_REGULATORS_S->VREGMAIN.DCDCEN  = (REGULATORS_VREGMAIN_DCDCEN_DCDCEN_Disabled << REGULATORS_VREGMAIN_DCDCEN_DCDCEN_Pos);
    NRF_REGULATORS_S->VREGH.DCDCEN     = (REGULATORS_VREGH_DCDCEN_DCDCEN_Disabled << REGULATORS_VREGH_DCDCEN_DCDCEN_Pos);

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

    NRF_IPC_S->INTENSET                          = 1 << DB_IPC_CHAN_RADIO_RX;
    NRF_IPC_S->SEND_CNF[DB_IPC_CHAN_REQ]         = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_S->RECEIVE_CNF[DB_IPC_CHAN_RADIO_RX] = 1 << DB_IPC_CHAN_RADIO_RX;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);

    // Start the network core
    release_network_core();

    if (callback) {
        _radio_callback = callback;
    }

    ipc_shared_data.radio.mode = mode;
    db_ipc_network_call(DB_IPC_RADIO_INIT_REQ);
}

void db_radio_set_frequency(uint8_t freq) {
    ipc_shared_data.radio.frequency = freq;
    db_ipc_network_call(DB_IPC_RADIO_FREQ_REQ);
}

void db_radio_set_channel(uint8_t channel) {
    ipc_shared_data.radio.channel = channel;
    db_ipc_network_call(DB_IPC_RADIO_CHAN_REQ);
}

void db_radio_set_power(uint8_t power){///////////////////////////////
    ipc_shared_data.radio.power = power;
    db_ipc_network_call(DB_IPC_RADIO_POWER_REQ);   
}

void db_radio_set_network_address(uint32_t addr) {
    ipc_shared_data.radio.addr = addr;
    db_ipc_network_call(DB_IPC_RADIO_ADDR_REQ);
}

void db_radio_tx(const uint8_t *tx_buffer, uint8_t length) {
    ipc_shared_data.radio.tx_pdu.length = length;
    memcpy((void *)ipc_shared_data.radio.tx_pdu.buffer, tx_buffer, length);
    db_ipc_network_call(DB_IPC_RADIO_TX_REQ);
}

void db_radio_rx(void) {
    db_ipc_network_call(DB_IPC_RADIO_RX_REQ);
}

void db_radio_tx_start(void){///////////////////////////////////////
    db_ipc_network_call(DB_IPC_RADIO_TX_IDLE_REQ);  
}

int8_t db_radio_rssi(void) {
    db_ipc_network_call(DB_IPC_RADIO_RSSI_REQ);
    return ipc_shared_data.radio.rssi;
}

void db_radio_disable(void) {
    db_ipc_network_call(DB_IPC_RADIO_DIS_REQ);
}

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX] = 0;
        if (_radio_callback) {
            mutex_lock();
            _radio_callback((uint8_t *)ipc_shared_data.radio.rx_pdu.buffer, ipc_shared_data.radio.rx_pdu.length);
            mutex_unlock();
        }
    }
}
