/**
 * @file rng_nrf5340_app.c
 * @addtogroup BSP
 *
 * @brief  nrf5340-app-specific definition of the "rng" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <nrf.h>
#include <stdint.h>

#include "ipc.h"
#include "rng.h"

//========================== functions =========================================

static void _network_call(ipc_event_type_t req, ipc_event_type_t ack) {
    ipc_shared_data.event    = req;
    NRF_IPC_S->TASKS_SEND[1] = 1;
    mutex_unlock();
    while (ipc_shared_data.event != ack) {}
}

//=========================== public ===========================================

void db_rng_init(void) {

    // RNG (address at 0x41009000 => periph ID is 8)
    NRF_SPU_S->PERIPHID[9].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

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

        while (ipc_shared_data.event != DB_IPC_NET_READY_REQ) {
            __WFE();
        }
    }

    mutex_lock();
    _network_call(DB_IPC_RNG_INIT_REQ, DB_IPC_RNG_INIT_ACK);
}

void db_rng_read(uint8_t *value) {
    mutex_lock();
    _network_call(DB_IPC_RNG_READ_REQ, DB_IPC_RNG_READ_ACK);
    *value = ipc_shared_data.rng.value;
}

//=========================== interrupt handlers ===============================

void IPC_IRQHandler(void) {
    if (NRF_IPC_S->EVENTS_RECEIVE[0]) {
        NRF_IPC_S->EVENTS_RECEIVE[0] = 0;
    }

    NVIC_ClearPendingIRQ(IPC_IRQn);
}
