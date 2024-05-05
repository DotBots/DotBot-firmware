/**
 * @file
 * @ingroup drv_tdma_client
 *
 * @brief  nrf5340-app-specific definition of the "tdma_client" drv module.
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

//=========================== variables ========================================

static tdma_client_cb_t _tdma_client_callback = NULL;

//=========================== public ===========================================

void db_tdma_client_init(tdma_client_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq) {
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

    // Store callback in a local variable
    if (callback) {
        _tdma_client_callback = callback;
    }

    // Store information in the shared data before sending it to the net-core
    ipc_shared_data.tdma_client.mode      = radio_mode;
    ipc_shared_data.tdma_client.frequency = radio_freq;

    // Initialice TDMA client drv in the net-core
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_INIT_REQ);
}

void db_tdma_client_set_table(const tdma_client_table_t *table) {

    // Copy the set table to the IPC chared data
    ipc_shared_data.tdma_client.table_set.frame_duration = table->frame_duration;
    ipc_shared_data.tdma_client.table_set.rx_start       = table->rx_start;
    ipc_shared_data.tdma_client.table_set.rx_duration    = table->rx_duration;
    ipc_shared_data.tdma_client.table_set.tx_start       = table->tx_start;
    ipc_shared_data.tdma_client.table_set.tx_duration    = table->tx_duration;
    // Request the network core to copy the data.
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_SET_TABLE_REQ);
}

void db_tdma_client_get_table(tdma_client_table_t *table) {

    // Request the network core to copy the table's data.
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_GET_TABLE_REQ);
    // Copy the get table from the IPC chared data
    table->frame_duration = ipc_shared_data.tdma_client.table_get.frame_duration;
    table->rx_start       = ipc_shared_data.tdma_client.table_get.rx_start;
    table->rx_duration    = ipc_shared_data.tdma_client.table_get.rx_duration;
    table->tx_start       = ipc_shared_data.tdma_client.table_get.tx_start;
    table->tx_duration    = ipc_shared_data.tdma_client.table_get.tx_duration;
}

void db_tdma_client_tx(const uint8_t *packet, uint8_t length) {
    ipc_shared_data.tdma_client.tx_pdu.length = length;
    memcpy((void *)ipc_shared_data.tdma_client.tx_pdu.buffer, packet, length);
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_TX_REQ);
}

void db_tdma_client_flush(void) {
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_FLUSH_REQ);
}

void db_tdma_client_empty(void) {
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_EMPTY_REQ);
}

db_tdma_registration_state_t db_tdma_client_get_status(void) {
    db_ipc_network_call(DB_IPC_TDMA_CLIENT_STATUS_REQ);
    return ipc_shared_data.tdma_client.registration_state;
}

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_RADIO_RX] = 0;
        if (_tdma_client_callback) {
            mutex_lock();
            _tdma_client_callback((uint8_t *)ipc_shared_data.tdma_client.rx_pdu.buffer, ipc_shared_data.tdma_client.rx_pdu.length);
            mutex_unlock();
        }
    }
}
