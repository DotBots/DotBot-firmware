/**
 * @file
 * @ingroup samples_bsp
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the Wheel revolution counter in the DotBot board.
 *
 * Load this program on your board. The LEDs should start blinking blue as the wheels are being turned.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include "board.h"
#include "rpm.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV (0)

//=========================== main =============================================

int main(void) {
    puts("RPM application");
    db_board_init();
    db_timer_init(TIMER_DEV);
    db_rpm_init();

    while (1) {
        rpm_values_t values = { 0 };
        db_rpm_get_values(&values);
        printf(
            "Left  - speed: %i, RPM: %i, RPS: %i\n",
            (int)values.left.speed,
            values.left.rpm,
            values.left.rps);
        printf(
            "Right - speed: %i, RPM: %i, RPS: %i\n",
            (int)values.right.speed,
            values.right.rpm,
            values.right.rps);
        db_timer_delay_ms(TIMER_DEV, 500);
    }

    return 0;
}
