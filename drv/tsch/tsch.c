/**
 * @file
 * @ingroup     drv_tsch
 *
 * @brief       Driver for Time-Slotted Channel Hopping (TSCH)
 *
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "tsch.h"
#include "scheduler.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"
#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "ipc.h"
#endif
//=========================== defines ==========================================

typedef struct {
    tsch_cb_t           callback;                              ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    application_type_t  default_radio_app;                     ///< Which application to use for registration and sync messages
} tsch_vars_t;

//=========================== variables ========================================

//========================== prototypes ========================================

//=========================== public ===========================================

void db_tsch_tick(void) {
    tsch_radio_event_t event = db_scheduler_tick();
    switch (event.radio_action) {
        // case TSCH_RADIO_ACTION_TX:
        //     db_radio_set_tx(event.frequency);
        //     break;
        // case TSCH_RADIO_ACTION_RX:
        //     db_radio_set_rx(event.frequency);
        //     break;
        // case TSCH_RADIO_ACTION_SLEEP:
        //     db_radio_set_sleep();
        //     break;
        default:
            break;
    }
}

//=========================== private ==========================================
