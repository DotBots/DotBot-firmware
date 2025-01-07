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

tsch_slot_timing_t tsch_default_slot_timing = {
    .rx_offset = 40 + 100, // Radio ramp-up time (40 us), + 100 us for any processing needed
    .rx_max = _TSCH_START_GUARD_TIME + _TSCH_PACKET_TOA_WITH_PADDING, // Guard time + Enough time to receive the maximum payload.
    .tx_offset = 40 + 100 + _TSCH_START_GUARD_TIME, // Same as rx_offset, plus the guard time.
    .tx_max = _TSCH_PACKET_TOA_WITH_PADDING, // Enough to transmit the maximum payload.
    .end_guard = 100, // Extra time at the end of the slot

    // receive slot is: rx_offset / rx_max / end_guard
    // transmit slot is: tx_offset / tx_max / end_guard
    .total_duration = 40 + 100 + _TSCH_START_GUARD_TIME + _TSCH_PACKET_TOA_WITH_PADDING + 100, // Total duration of the slot
};

typedef struct {
    tsch_cb_t application_callback; ///< Function pointer, stores the application callback
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

void db_tsch_init(tsch_cb_t application_callback) {
    // initialize the high frequency clock
    db_timer_hf_init(TSCH_TIMER_DEV);

    // initialize the radio
    db_radio_init(&_tsch_callback, DB_RADIO_BLE_2MBit);  // set the radio callback to our tsch catch function

    // Save the application callback to use in our interruption
    _tsch_vars.application_callback = application_callback;

    _tsch_vars.device_id = db_device_id();

    // NOTE: assume the scheduler has already been initialized by the application

    // set slot total duration
    tsch_default_slot_timing.total_duration = tsch_default_slot_timing.rx_offset + tsch_default_slot_timing.rx_max + tsch_default_slot_timing.end_guard;

    // start the ticking immediately
    db_timer_hf_set_oneshot_us(TSCH_TIMER_DEV, TSCH_TIMER_SLOT_CHANNEL, 100, _timer_tsch_slot_handler);
}

//=========================== private ==========================================


// TODO: manipulate the _tsch_vars.state


void _timer_tsch_slot_handler(void) {
    // FIXME: for some reason, it just receives (beacon) packets on frequency 26, and it is always "CRC error"

    tsch_radio_event_t event = db_scheduler_tick();
    printf("Event %c:   %c, %d    Slot duration: %d\n", event.slot_type, event.radio_action, event.frequency, tsch_default_slot_timing.total_duration); // FIXME: only for debugging, remove before merge

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
    db_timer_hf_set_oneshot_us(TSCH_TIMER_DEV, TSCH_TIMER_SLOT_CHANNEL, tsch_default_slot_timing.total_duration, _timer_tsch_slot_handler);
}

static void _tsch_callback(uint8_t *packet, uint8_t length) {
    printf("Received packet of length %d\n", length);

    // Check if the packet is big enough to have a header
    if (length < sizeof(protocol_header_t)) {
        return;
    }

    // Check if it is a beacon
    if (packet[1] == TSCH_PACKET_TYPE_BEACON) {
        // _tsch_handle_beacon(packet, length);
    } else if (_tsch_vars.application_callback) {
        _tsch_vars.application_callback(packet, length);
    }
}
