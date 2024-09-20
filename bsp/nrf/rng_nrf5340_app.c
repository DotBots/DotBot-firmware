/**
 * @file
 * @ingroup bsp_rng
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
#include "tz.h"

//========================== variables =========================================

extern volatile __attribute__((section(".shared_data"))) ipc_shared_data_t ipc_shared_data;

//=========================== public ===========================================

void db_rng_init(void) {
    // RNG (address at 0x41009000 => periph ID is 9)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_RNG);

    // IPC (address at 0x41012000 => periph ID is 18)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_IPC);

    // APPMUTEX (address at 0x41030000 => periph ID is 48)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_APPMUTEX);

    // Define RAMREGION 2 (0x20004000 to 0x20005FFF, e.g 8KiB) as non secure. It's used to share data between cores
    db_configure_ram_non_secure(2, 1);

    NRF_IPC_S->SEND_CNF[DB_IPC_CHAN_REQ] = 1 << DB_IPC_CHAN_REQ;

    NVIC_EnableIRQ(IPC_IRQn);
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);

    // Start the network core
    release_network_core();

    db_ipc_network_call(DB_IPC_RNG_INIT_REQ);
}

void db_rng_read(uint8_t *value) {
    db_ipc_network_call(DB_IPC_RNG_READ_REQ);
    *value = ipc_shared_data.rng.value;
}
