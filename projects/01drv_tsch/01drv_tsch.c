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



int main(void) {
    puts("TSCH test application");

    while (1) {
        __WFE();
    }
}
