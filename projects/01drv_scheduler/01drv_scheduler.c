/**
 * @file
 * @ingroup     drv_scheduler
 *
 * @brief       Example on how to use the TSCH scheduler
 *
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "scheduler.h"
#include "tsch.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"

#define IS_DOTBOT
//#define TSCH_DEFAULT_SLOT_DURATION_US 1000 * 1000
#define TSCH_DEFAULT_SLOT_DURATION_US 2024

/* Very simple test schedule */
schedule_t schedule_test = {
    .id = 10, // make sure it doesn't collide
    .max_nodes = 2,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 5,
    .cells = {
        // Only downlink slots
        {'B', 0, NULL},
        {'S', 1, NULL},
        {'D', 2, NULL},
        {'U', 3, NULL},
        {'U', 4, NULL},
    }
};

extern schedule_t schedule_minuscule;

int main(void) {
    // initialize high frequency timer
    db_timer_hf_init(TSCH_TIMER_DEV);

    // initialize schedule
    schedule_t schedule = schedule_minuscule;
    node_type_t node_type = NODE_TYPE_DOTBOT;
    db_scheduler_init(node_type, &schedule);

    printf("Device of type %c and id %llx is using schedule %d\n\n", node_type, db_device_id(), schedule.id);

    // loop n_slotframes*n_cells times and make the scheduler tick
    // also, try to assign and deassign uplink cell at specific slotframes
    size_t n_slotframes = 4;
    for (size_t j = 0; j < n_slotframes; j++) {
        for (size_t i = 0; i < schedule.n_cells; i++) {
            tsch_radio_event_t event = db_scheduler_tick();
            printf("Event %c:   %c, %d, %d\n", event.slot_type, event.radio_action, event.frequency, event.duration_us);

            // sleep for the duration of the slot
            db_timer_hf_delay_us(TSCH_TIMER_DEV, event.duration_us);
        }
        puts(".");
        if (j == 0 && !db_scheduler_assign_next_available_uplink_cell(db_device_id())) { // try to assign at the end of first slotframe
            printf("Failed to assign uplink cell\n");
            return 1;
        } else if (j == n_slotframes-2 && !db_scheduler_deassign_uplink_cell(db_device_id())) { // try to deassign at the end of the second-to-last slotframe
            printf("Failed to deassign uplink cell\n");
            return 1;
        }
    }

    while (1) {
        __WFE();
    }
}
