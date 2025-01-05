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

#define N_SCHEDULES 1 + 2 // account for the schedule that can be passed by the application during initialization

/* Schedule with 11 slots, supporting up to 5 nodes */
schedule_t schedule_minuscule = {
    .id = 32,
    .max_nodes = 5,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = 2024,
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
    .slot_duration_us = 2024,
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

