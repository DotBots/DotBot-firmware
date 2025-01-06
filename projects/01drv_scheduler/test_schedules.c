#include "scheduler.h"

#define IS_DOTBOT
// #define TSCH_DEFAULT_SLOT_DURATION_US 2024
#define TSCH_DEFAULT_SLOT_DURATION_US 1000 * 1000

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

/* Uplink only test schedule */
schedule_t schedule_all_uplink = {
    .id = 10, // make sure it doesn't collide
    .max_nodes = 2,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 5,
    .cells = {
        // Only downlink slots
        {'U', 0, NULL},
        {'U', 1, NULL},
        {'U', 2, NULL},
        {'U', 3, NULL},
        {'U', 4, NULL},
    }
};

/* Downlink only test schedule */
schedule_t schedule_all_downlink = {
    .id = 10, // make sure it doesn't collide
    .max_nodes = 2,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .slot_duration_us = TSCH_DEFAULT_SLOT_DURATION_US,
    .n_cells = 5,
    .cells = {
        // Only downlink slots
        {'D', 0, NULL},
        {'D', 1, NULL},
        {'D', 2, NULL},
        {'D', 3, NULL},
        {'D', 4, NULL},
    }
};
