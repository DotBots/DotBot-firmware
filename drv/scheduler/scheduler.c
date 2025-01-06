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

#include "scheduler.h"
#include "tsch.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"
#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "ipc.h"
#endif

#include "all_schedules.c"

//=========================== defines ==========================================


//=========================== variables ========================================

static const uint8_t _ble_chan_to_freq[40] = {
    4, 6, 8,
    10, 12, 14, 16, 18,
    20, 22, 24, 28,
    30, 32, 34, 36, 38,
    40, 42, 44, 46, 48,
    50, 52, 54, 56, 58,
    60, 62, 64, 66, 68,
    70, 72, 74, 76, 78,
    2, 26, 80  // Advertising channels
};

typedef struct {
    // counters and indexes
    uint64_t asn; // absolute slot number

    schedule_t *active_schedule_ptr; // pointer to the currently active schedule
    uint32_t slotframe_counter; // used to cycle beacon frequencies through slotframes (when listening for beacons at uplink slots)
    size_t current_beacon_cell_index;

    // static data
    schedule_t available_schedules[N_SCHEDULES];
    size_t available_schedules_len;
} schedule_vars_t;

static schedule_vars_t _schedule_vars = { 0 };

//========================== prototypes ========================================


//=========================== public ===========================================

void db_scheduler_init(schedule_t *application_schedule) {
    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_minuscule;
    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_tiny;
    if (application_schedule != NULL) {
        _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = *application_schedule;
        _schedule_vars.active_schedule_ptr = application_schedule;
    }
}

bool db_scheduler_set_schedule(uint8_t schedule_id) {
    for (size_t i = 0; i < N_SCHEDULES; i++) {
        if (_schedule_vars.available_schedules[i].id == schedule_id) {
            _schedule_vars.active_schedule_ptr = &_schedule_vars.available_schedules[i];
            return true;
        }
    }
    return false;
}

tsch_radio_event_t db_scheduler_tick(void) {
    schedule_t active_schedule = *_schedule_vars.active_schedule_ptr;

    // get the current cell
    size_t cell_index = _schedule_vars.asn % active_schedule.n_cells;
    cell_t current_cell = active_schedule.cells[cell_index];

    tsch_radio_event_t radio_event = {
        .radio_action = TSCH_RADIO_ACTION_SLEEP,
        .duration_us = active_schedule.slot_duration_us,
        .frequency = 0
    };

    radio_event.frequency = db_scheduler_get_frequency(current_cell.type, _schedule_vars.asn, current_cell.channel_offset);

    switch (current_cell.type) {
        case SLOT_TYPE_BEACON:
        case SLOT_TYPE_DOWNLINK:
            radio_event.radio_action = TSCH_RADIO_ACTION_RX;
            break;
        case SLOT_TYPE_SHARED_UPLINK:
            radio_event.radio_action = TSCH_RADIO_ACTION_TX;
            break;
        case SLOT_TYPE_UPLINK:
            if (current_cell.assigned_node_id != NULL) {
                radio_event.radio_action = TSCH_RADIO_ACTION_TX;
            }
            break;
        default:
            break;
    }

// #define IS_DOTBOT
#ifdef IS_DOTBOT
    if (current_cell.type == SLOT_TYPE_UPLINK && current_cell.assigned_node_id == NULL) {
        // listen for beacons using the same frequency during a whole slotframe
        radio_event.radio_action = TSCH_RADIO_ACTION_RX;
        size_t beacon_channel = TSCH_N_BLE_REGULAR_FREQUENCIES + (_schedule_vars.slotframe_counter % TSCH_N_BLE_ADVERTISING_FREQUENCIES);
        radio_event.frequency = _ble_chan_to_freq[beacon_channel];
    }
#endif

    // if the slotframe wrapped, keep track of how many slotframes have passed (used to cycle beacon frequencies)
    if (cell_index == 0) {
        _schedule_vars.slotframe_counter++;
    }

    // increment ASN so that (1) nodes are in sync and (2) next time we get the next cell
    _schedule_vars.asn++;

    return radio_event;
}

uint8_t db_scheduler_get_frequency(slot_type_t slot_type, uint64_t asn, uint8_t channel_offset) {
    if (slot_type == SLOT_TYPE_BEACON) {
        // special handling in case the cell is a beacon
        size_t beacon_channel = TSCH_N_BLE_REGULAR_FREQUENCIES + (_schedule_vars.current_beacon_cell_index++ % TSCH_N_BLE_ADVERTISING_FREQUENCIES);
        uint8_t freq = _ble_chan_to_freq[beacon_channel];
        return freq;
    } else {
        // As per RFC 7554:
        //   frequency = F {(ASN + channelOffset) mod nFreq}
        size_t freq_index = (asn + channel_offset) % TSCH_N_BLE_REGULAR_FREQUENCIES;
        return _ble_chan_to_freq[freq_index];
    }
}

//=========================== private ==========================================
