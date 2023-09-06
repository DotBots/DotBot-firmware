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
#include <stdbool.h>
#include <stdint.h>

#include "ipc.h"
#include "rng.h"

//========================== variables =========================================

static bool _ack_received[] = {
    [DB_IPC_NET_READY_ACK] = false,
    [DB_IPC_RNG_INIT_ACK]  = false,
    [DB_IPC_RNG_READ_ACK]  = false,
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
};

//=========================== public ===========================================

void db_rng_init(void) {

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

    NRF_IPC_S->INTENSET                     = 1 << DB_IPC_CHAN_ACK;
    NRF_IPC_S->SEND_CNF[DB_IPC_CHAN_REQ]    = 1 << DB_IPC_CHAN_REQ;
    NRF_IPC_S->RECEIVE_CNF[DB_IPC_CHAN_ACK] = 1 << DB_IPC_CHAN_ACK;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);

    // Start the network core
    if (NRF_RESET_S->NETWORK.FORCEOFF != 0) {
        db_timer_hf_init();
        *(volatile uint32_t *)0x50005618ul = 1ul;
        NRF_RESET_S->NETWORK.FORCEOFF      = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
        db_timer_hf_delay_us(5);  // Wait for at least five microseconds
        NRF_RESET_S->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Hold << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
        db_timer_hf_delay_us(5);  // Wait for at least one microsecond
        NRF_RESET_S->NETWORK.FORCEOFF      = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);
        *(volatile uint32_t *)0x50005618ul = 0ul;
        _network_call(DB_IPC_NONE, DB_IPC_NET_READY_ACK);
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
    if (NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_ACK]) {
        NRF_IPC_S->EVENTS_RECEIVE[DB_IPC_CHAN_ACK] = 0;
        mutex_lock();
        _ack_received[ipc_shared_data.event] = true;
        mutex_unlock();
    }
}
