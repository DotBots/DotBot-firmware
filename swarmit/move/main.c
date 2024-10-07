/**
 * @file
 * @defgroup swarmit_move    Move application on top of SwarmIT
 * @ingroup swarmit
 * @brief   This application uses the move API to make the robot move
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <nrf.h>
#include "move.h"
#include "timer.h"
#include "gpio.h"
#include "board_config.h"

//=========================== swarmit ==========================================

void reload_wdt0(void);

//=========================== main =============================================

int main(void) {
    db_timer_init(1);
    db_timer_set_periodic_ms(1, 0, 500, &reload_wdt0);

    db_gpio_init(&db_led1, DB_GPIO_OUT);

    db_move_init();
    db_move_straight(200, 60);
    db_move_rotate(90, 60);
    db_move_straight(200, 60);
    db_move_rotate(90, 60);
    db_move_straight(200, 60);
    db_move_rotate(90, 60);
    db_move_straight(200, 60);
    db_move_rotate(90, 60);

    while (1) {
        db_gpio_toggle(&db_led1);
        db_timer_delay_ms(1, 250);
    }
}
