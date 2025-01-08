/**
 * @file
 * @ingroup samples_bsp
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the motor driver in the DotBot board.
 *
 * Load this program on your board. The right wheel should spin forwards and then backward.
 * Afterwards the left wheel also turns forward and then backward.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdint.h>
#include <nrf.h>
// Include BSP packages
#include "board.h"
#include "motors.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV (0)

//=========================== main =============================================

int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Initialize the timer
    db_timer_init(TIMER_DEV);

    // Configure Motors
    db_motors_init();

    while (1) {
        // Move forward
        for (uint8_t speed = 50; speed < 100; speed++) {
            db_motors_set_speed(speed, speed);
            db_timer_delay_ms(TIMER_DEV, 20);
        }

        // Move backward
        for (uint8_t speed = 50; speed < 100; speed++) {
            db_motors_set_speed(speed * -1, speed * -1);
            db_timer_delay_ms(TIMER_DEV, 20);
        }
    }
}
