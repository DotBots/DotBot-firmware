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
#include "scheduler.h"

#define TSCH_N_SCHEDULES 1 + 2 // account for the schedule that can be passed by the application during initialization

#ifndef TSCH_DEFAULT_SLOT_DURATION_US
#define TSCH_DEFAULT_SLOT_DURATION_US 2024
// #define TSCH_DEFAULT_SLOT_DURATION_US 1000 * 1000
#endif

/* Schedule with 11 slots, supporting up to 5 nodes */
schedule_t schedule_minuscule = {
    .id = 6,
    .max_nodes = 5,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    // .slot_duration_us = 2024,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 11,
    .cells = {
        // Begin with beacon cells. They use their own channel offsets and frequencies.
        {'B', 0, NULL},
        {'B', 1, NULL},
        {'B', 2, NULL},
        // Continue with regular cells.
        {'S', 6, NULL},
        {'D', 3, NULL},
        {'U', 5, NULL},
        {'U', 1, NULL},
        {'D', 4, NULL},
        {'U', 0, NULL},
        {'U', 7, NULL},
        {'U', 2, NULL}
    }
};

/* Schedule with 17 slots, supporting up to 11 nodes */
schedule_t schedule_tiny = {
    .id = 5,
    .max_nodes = 11,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 17,
    .cells = {
        // Begin with beacon cells. They use their own channel offsets and frequencies.
        {'B', 0, NULL},
        {'B', 1, NULL},
        {'B', 2, NULL},
        // Continue with regular cells.
        {'S', 2, NULL},
        {'D', 5, NULL},
        {'U', 6, NULL},
        {'U', 13, NULL},
        {'U', 7, NULL},
        {'U', 0, NULL},
        {'D', 4, NULL},
        {'U', 10, NULL},
        {'U', 12, NULL},
        {'U', 1, NULL},
        {'U', 11, NULL},
        {'U', 8, NULL},
        {'U', 3, NULL},
        {'U', 9, NULL}
    }
};

/* Schedule with 41 slots, supporting up to 29 nodes */
schedule_t schedule_small = {
    .id = 4,
    .max_nodes = 29,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 41,
    .cells = {
        {'B', 0, NULL},
        {'B', 1, NULL},
        {'B', 2, NULL},
        {'S', 36, NULL},
        {'D', 20, NULL},
        {'U', 13, NULL},
        {'U', 27, NULL},
        {'U', 29, NULL},
        {'U', 9, NULL},
        {'D', 0, NULL},
        {'U', 4, NULL},
        {'U', 33, NULL},
        {'U', 3, NULL},
        {'U', 30, NULL},
        {'U', 31, NULL},
        {'S', 22, NULL},
        {'D', 15, NULL},
        {'U', 11, NULL},
        {'U', 16, NULL},
        {'U', 24, NULL},
        {'U', 21, NULL},
        {'D', 2, NULL},
        {'U', 19, NULL},
        {'U', 10, NULL},
        {'U', 25, NULL},
        {'U', 34, NULL},
        {'U', 14, NULL},
        {'S', 28, NULL},
        {'D', 32, NULL},
        {'U', 1, NULL},
        {'U', 5, NULL},
        {'U', 18, NULL},
        {'U', 7, NULL},
        {'D', 23, NULL},
        {'U', 12, NULL},
        {'U', 17, NULL},
        {'U', 6, NULL},
        {'U', 35, NULL},
        {'U', 8, NULL},
        {'U', 37, NULL},
        {'U', 26, NULL},
    }
};