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

#define IS_DOTBOT

/* Very simple test schedule */
schedule_t schedule_test = {
    .id = 10, // make sure it doesn't collide
    .max_nodes = 0,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = 2024,
    .n_cells = 3,
    .cells = {
        // Only downlink slots
        {'D', 0, NULL},
        {'D', 1, NULL},
        {'D', 2, NULL},
    }
};

int main(void) {
    puts("Scheduler test application");

    // initialize high frequency timer
    db_timer_hf_init(TSCH_TIMER_DEV);

    db_scheduler_init(&schedule_test);
    // db_scheduler_set_schedule(5);

    uint8_t freq = db_scheduler_get_frequency(SLOT_TYPE_SHARED_UPLINK, 0, 0);
    printf("Frequency: %d\n", freq);

    while (1) {
        tsch_radio_event_t event = db_scheduler_tick();
        printf("Event: %d, %d, %d\n", event.radio_action, event.frequency, event.duration_us);

        // sleep for the duration of the slot
        db_timer_hf_delay_us(TSCH_TIMER_DEV, event.duration_us);

        __WFE();
    }
}
