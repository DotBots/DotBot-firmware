/**
 * @file
 * @defgroup swarmit_motors Motors application on top of SwarmIT
 * @ingroup swarmit
 * @brief   This application controls the motors of the robot
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdint.h>

#include "board.h"
#include "motors.h"
#include "timer.h"

//=========================== swarmit ==========================================

void swarmit_reload_wdt0(void);

//=========================== defines ==========================================

#define TIMER_DEV (0)

//=========================== main =============================================

int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Initialize the timer
    db_timer_init(TIMER_DEV);

    db_timer_init(1);
    db_timer_set_periodic_ms(1, 0, 500, &swarmit_reload_wdt0);

    // Configure Motors
    db_motors_init();

    while (1) {
        // Move forward
        for (uint8_t speed = 50; speed < 80; speed++) {
            db_motors_set_speed(speed, speed);
            db_timer_delay_ms(TIMER_DEV, 30);
        }

        // Move backward
        for (uint8_t speed = 50; speed < 80; speed++) {
            db_motors_set_speed(speed * -1, speed * -1);
            db_timer_delay_ms(TIMER_DEV, 30);
        }

        // Spin
        db_motors_set_speed(-70, 70);
        db_timer_delay_ms(TIMER_DEV, 500);

        // Spin back
        db_motors_set_speed(70, -70);
        db_timer_delay_ms(TIMER_DEV, 500);
    }
}
