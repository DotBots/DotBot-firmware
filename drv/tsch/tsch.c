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

//=========================== variables ========================================

typedef enum {
    TSCH_STATE_WAIT_TO_RX,
    TSCH_STATE_STARTED_RX,
    TSCH_STATE_DONE_RX,
    // etc.
} tsch_state_t; // TODO: actually use this state in the handlers below

typedef struct {
    tsch_cb_t callback; ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    uint64_t device_id; ///< Device ID
    tsch_state_t state; ///< State of the TSCH state machine
} tsch_vars_t;

static tsch_vars_t _tsch_vars = { 0 };

uint8_t default_packet[] = {
    0x08, // size of the packet
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};

//========================== prototypes ========================================

/**
 * @brief Interrupt handler for the slot-wise ticking of the TSCH scheduler.
 */
static void _timer_tsch_slot_handler(void);

/*
* @brief Callback function passed to the radio driver.
*/
static void _tsch_callback(uint8_t *packet, uint8_t length);

//=========================== public ===========================================

void db_tsch_init(tsch_cb_t callback) {
    // initialize the high frequency clock
    db_timer_hf_init(TSCH_TIMER_DEV);

    // initialize the radio
    db_radio_init(&_tsch_callback, DB_RADIO_BLE_2MBit);  // set the radio callback to our tsch catch function

    // Save the user callback to use in our interruption
    _tsch_vars.callback = callback;

    _tsch_vars.device_id = db_device_id();

    // NOTE: assume the scheduler has already been initialized by the application

    // start the ticking immediately
    db_timer_hf_set_oneshot_us(TSCH_TIMER_DEV, TSCH_TIMER_SLOT_CHANNEL, 100, _timer_tsch_slot_handler);
}

//=========================== private ==========================================


// TODO: manipulate the _tsch_vars.state


void _timer_tsch_slot_handler(void) {
    // FIXME: for some reason, it just receives (beacon) packets on frequency 26, and it is always "CRC error"

    tsch_radio_event_t event = db_scheduler_tick();
    printf("Event %c:   %c, %d, %d\n", event.slot_type, event.radio_action, event.frequency, event.duration_us); // FIXME: only for debugging, remove before merge

    switch (event.radio_action) {
        case TSCH_RADIO_ACTION_TX:
            db_radio_disable();
            db_radio_set_frequency(event.frequency);
            // TODO: send a real packet:
            // - from the application queue
            // - a beacon, if it is a beacon slot and the node is a gateway
            db_radio_tx(default_packet, sizeof(default_packet));
            break;
        case TSCH_RADIO_ACTION_RX:
            // TODO: if this is a DotBot, implement re-synchronization logic
            db_radio_disable();
            db_radio_set_frequency(event.frequency);
            db_radio_rx();
            break;
        case TSCH_RADIO_ACTION_SLEEP:
            db_radio_disable();
            break;
    }

    // schedule the next tick
    // FIXME: compensate for the time spent in the instructions above. For now, just using 'event.duration_us' will do.
    db_timer_hf_set_oneshot_us(TSCH_TIMER_DEV, TSCH_TIMER_SLOT_CHANNEL, event.duration_us, _timer_tsch_slot_handler);
}

static void _tsch_callback(uint8_t *packet, uint8_t length) {
    uint8_t *ptk_ptr = packet;
    protocol_header_t *header = (protocol_header_t *)ptk_ptr;

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _tsch_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    switch (header->type) {
        case TSCH_PACKET_TYPE_BEACON:
        {
            // ...
        } break;

        // This is not a TSCH packet, send to the user callback
        default:
        {
            if (_tsch_vars.callback) {
                _tsch_vars.callback(packet, length);
            }
        }
    }
}
