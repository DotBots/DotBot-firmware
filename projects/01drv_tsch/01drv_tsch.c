/**
 * @file
 * @ingroup     drv_tsch
 *
 * @brief       Example on how to use the TSCH driver
 *
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "tsch.h"
#include "scheduler.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"

/* Very simple test schedule */
schedule_t schedule_test = {
    .id = 32, // make sure it doesn't collide
    .max_nodes = 0,
    .backoff_n_min = 5,
    .backoff_n_max = 9,
    .n_cells = 5,
    .cells = {
        //{'B', 0, NULL},
        //{'S', 1, NULL},
        //{'D', 2, NULL},
        //{'U', 3, NULL},
        //{'U', 4, NULL},

        {'S', 0, NULL},
        {'B', 1, NULL},
        {'B', 2, NULL},
        {'B', 3, NULL},
        {'B', 4, NULL},

        //{'U', 0, NULL},
        //{'U', 1, NULL},
        //{'U', 2, NULL},
        //{'U', 3, NULL},
        //{'U', 4, NULL},
    }
};

extern schedule_t schedule_minuscule, schedule_small, schedule_huge, schedule_only_beacons, schedule_only_beacons_optimized_scan;

static void radio_callback(uint8_t *packet, uint8_t length);

int main(void) {
    // initialize schedule

    //schedule_t schedule = schedule_only_beacons;
    //node_type_t node_type = NODE_TYPE_GATEWAY;
    schedule_t schedule = schedule_huge;
    node_type_t node_type = NODE_TYPE_DOTBOT;

    db_scheduler_init(node_type, &schedule);
    printf("\n==== Device of type %c and id %llx is using schedule %d ====\n\n", node_type, db_device_id(), schedule.id);

    // initialize the TSCH driver
    //tsch_default_slot_timing.end_guard = 1000 * 1000; // add an extra second of delay.
    db_tsch_init(node_type, radio_callback);
    printf("Slot total duration: %d us\n", tsch_default_slot_timing.total_duration);

    while (1) {
        __WFE();
    }
}

static void radio_callback(uint8_t *packet, uint8_t length) {
    (void) packet;
    (void) length;
    //printf("Received packet of length %d\n", length);
    //for (uint8_t i = 0; i < length; i++) {
    //    printf("%02x ", packet[i]);
    //}
    //puts("");
}
