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
    node_type_t node_type; // whether the node is a gateway or a dotbot

    // counters and indexes
    uint64_t asn; // absolute slot number
    schedule_t *active_schedule_ptr; // pointer to the currently active schedule
    uint32_t slotframe_counter; // used to cycle beacon frequencies through slotframes (when listening for beacons at uplink slots)

    // static data
    schedule_t available_schedules[TSCH_N_SCHEDULES];
    size_t available_schedules_len;
} schedule_vars_t;

static schedule_vars_t _schedule_vars = { 0 };

//========================== prototypes ========================================

// Compute the radio action when the node is a gateway
void _compute_gateway_action(cell_t cell, tsch_radio_event_t *radio_event);

// Compute the radio action when the node is a dotbot
void _compute_dotbot_action(cell_t cell, tsch_radio_event_t *radio_event);

//=========================== public ===========================================

void db_scheduler_init(node_type_t node_type, schedule_t *application_schedule) {
    _schedule_vars.node_type = node_type;

    if (_schedule_vars.available_schedules_len == TSCH_N_SCHEDULES) return; // FIXME: this is just to simplify debugging (allows calling init multiple times)

    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_only_beacons;
    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_only_beacons_optimized_scan;

    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_minuscule;
    _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = schedule_tiny;

    if (application_schedule != NULL) {
        _schedule_vars.available_schedules[_schedule_vars.available_schedules_len++] = *application_schedule;
        _schedule_vars.active_schedule_ptr = application_schedule;
    }
}

bool db_scheduler_set_schedule(uint8_t schedule_id) {
    for (size_t i = 0; i < TSCH_N_SCHEDULES; i++) {
        if (_schedule_vars.available_schedules[i].id == schedule_id) {
            _schedule_vars.active_schedule_ptr = &_schedule_vars.available_schedules[i];
            return true;
        }
    }
    return false;
}

bool db_scheduler_assign_next_available_uplink_cell(uint64_t node_id) {
    for (size_t i = 0; i < _schedule_vars.active_schedule_ptr->n_cells; i++) {
        cell_t *cell = &_schedule_vars.active_schedule_ptr->cells[i];
        if (cell->type == SLOT_TYPE_UPLINK && cell->assigned_node_id == NULL) {
            cell->assigned_node_id = node_id;
            return true;
        }
    }
    return false;
}

bool db_scheduler_deassign_uplink_cell(uint64_t node_id) {
    for (size_t i = 0; i < _schedule_vars.active_schedule_ptr->n_cells; i++) {
        cell_t *cell = &_schedule_vars.active_schedule_ptr->cells[i];
        if (cell->type == SLOT_TYPE_UPLINK && cell->assigned_node_id == node_id) {
            cell->assigned_node_id = NULL;
            return true;
        }
    }
    return false;
}

tsch_radio_event_t db_scheduler_tick(uint64_t asn) {
    // get the current cell
    size_t cell_index = asn % (_schedule_vars.active_schedule_ptr)->n_cells;
    cell_t cell = (_schedule_vars.active_schedule_ptr)->cells[cell_index];

    tsch_radio_event_t radio_event = {
        .radio_action = TSCH_RADIO_ACTION_SLEEP,
        .frequency = db_scheduler_get_frequency(cell.type, asn, cell.channel_offset),
        .slot_type = cell.type, // FIXME: only for debugging, remove before merge
    };
    if (_schedule_vars.node_type == NODE_TYPE_GATEWAY) {
        _compute_gateway_action(cell, &radio_event);
    } else {
        _compute_dotbot_action(cell, &radio_event);
    }

    // if the slotframe wrapped, keep track of how many slotframes have passed (used to cycle beacon frequencies)
    if (asn != 0 && cell_index == 0) {
        _schedule_vars.slotframe_counter++;
    }

    return radio_event;
}

uint8_t db_scheduler_get_frequency(slot_type_t slot_type, uint64_t asn, uint8_t channel_offset) {
    if (slot_type == SLOT_TYPE_BEACON) {
        // special handling in case the cell is a beacon
        size_t beacon_channel = TSCH_N_BLE_REGULAR_FREQUENCIES + (asn % TSCH_N_BLE_ADVERTISING_FREQUENCIES);
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

void _compute_gateway_action(cell_t cell, tsch_radio_event_t *radio_event) {
    switch (cell.type) {
        case SLOT_TYPE_BEACON:
        case SLOT_TYPE_DOWNLINK:
            radio_event->radio_action = TSCH_RADIO_ACTION_TX;
            break;
        case SLOT_TYPE_SHARED_UPLINK:
        case SLOT_TYPE_UPLINK:
            radio_event->radio_action = TSCH_RADIO_ACTION_RX;
            break;
    }
}

void _compute_dotbot_action(cell_t cell, tsch_radio_event_t *radio_event) {
    switch (cell.type) {
        case SLOT_TYPE_BEACON:
        case SLOT_TYPE_DOWNLINK:
            radio_event->radio_action = TSCH_RADIO_ACTION_RX;
            break;
        case SLOT_TYPE_SHARED_UPLINK:
            // TODO: implement backoff algorithm
            radio_event->radio_action = TSCH_RADIO_ACTION_TX;
            break;
        case SLOT_TYPE_UPLINK:
            if (cell.assigned_node_id == db_device_id()) {
                radio_event->radio_action = TSCH_RADIO_ACTION_TX;
            } else {
#ifdef TSCH_LISTEN_DURING_UNSCHEDULED_UPLINK
                // OPTIMIZATION: listen for beacons during unassigned uplink slot
                // listen to the same beacon frequency for a whole slotframe
                radio_event->radio_action = TSCH_RADIO_ACTION_RX;
                size_t beacon_channel = TSCH_N_BLE_REGULAR_FREQUENCIES + (_schedule_vars.slotframe_counter % TSCH_N_BLE_ADVERTISING_FREQUENCIES);
                radio_event->frequency = _ble_chan_to_freq[beacon_channel];
#endif
            }
            break;
        default:
            break;
    }
}
