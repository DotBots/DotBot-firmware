/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the QDEC api.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdbool.h>
#include <stdio.h>
#include "board_config.h"
#include "qdec.h"
#include "timer.h"

//=========================== defines ==========================================

#define QDEC_LEFT  0
#define QDEC_RIGHT 1
#define TIMER_DEV  0

typedef struct {
    bool left_overflow;
    bool right_overflow;
} qdec_vars_t;

//=========================== variables ========================================

static const qdec_conf_t qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t qdec_right = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

static qdec_vars_t _qdec_vars = {
    .left_overflow  = false,
    .right_overflow = false,
};

//=========================== callbacks ========================================

static void _callback(void *ctx) {
    *(bool *)ctx = true;
}

//=========================== main =============================================

int main(void) {
    db_qdec_init(QDEC_LEFT, &qdec_left, _callback, (void *)&_qdec_vars.left_overflow);
    db_qdec_init(QDEC_RIGHT, &qdec_right, _callback, (void *)&_qdec_vars.right_overflow);

    db_timer_init(TIMER_DEV);
    printf("ACC left: %i\n", db_qdec_read_and_clear(QDEC_LEFT));
    printf("ACC right: %i\n", db_qdec_read_and_clear(QDEC_RIGHT));

    while (1) {
        if (_qdec_vars.left_overflow) {
            puts("ACC left overflow");
            _qdec_vars.left_overflow = false;
        }
        if (_qdec_vars.right_overflow) {
            puts("ACC right overflow");
            _qdec_vars.right_overflow = false;
        }
        printf("ACC left: %i\n", db_qdec_read(QDEC_LEFT));
        printf("ACC right: %i\n", db_qdec_read(QDEC_RIGHT));
        db_timer_delay_s(TIMER_DEV, 1);
    }
}
