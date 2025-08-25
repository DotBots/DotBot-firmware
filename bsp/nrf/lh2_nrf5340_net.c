/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF5340-net-specific definition of the "lh2" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include "gpio.h"
#include "lh2.h"

//=========================== public ===========================================

void db_lh2_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e) {
    (void)lh2;
    (void)gpio_d;
    (void)gpio_e;
}

void db_lh2_start(void) {
}

void db_lh2_stop(void) {
}

void db_lh2_reset(db_lh2_t *lh2) {
    (void)lh2;
}

void db_lh2_process_raw_data(db_lh2_t *lh2) {
    (void)lh2;
}

void db_lh2_process_location(db_lh2_t *lh2) {
    (void)lh2;
}

void db_lh2_handle_isr(void) {
}
