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
#include <stdbool.h>

#include "board.h"
#include "board_config.h"
#include "motors.h"
#include "timer.h"
#include "gpio.h"

//=========================== swarmit ==========================================

void swarmit_keep_alive(void);
void swarmit_localization_handle_isr(void);

static bool keep_alive_enabled = true;

void app_swarmit_keep_alive(void) {
    if (keep_alive_enabled) {
        swarmit_keep_alive();
    }
}

//=========================== defines ==========================================

#define TIMER_DEV (0)

//=========================== main =============================================

int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    db_gpio_init(&db_led1, DB_GPIO_OUT);

    // Initialize the timer
    db_timer_init(TIMER_DEV);

    db_timer_init(1);
    keep_alive_enabled = true;
    db_timer_set_periodic_ms(1, 0, 200, &app_swarmit_keep_alive);

    // Configure Motors
    db_motors_init();

    // Blink for 2 seconds
    for (int i = 0; i < 16; i++) {
        db_gpio_toggle(&db_led1);
        db_timer_delay_ms(1, 125);
    }
    db_gpio_set(&db_led1);
    db_timer_delay_ms(1, 1000);

    // Spin
    db_motors_set_speed(-70, 70);
    db_timer_delay_ms(TIMER_DEV, 2000);

    // Stop
    db_motors_set_speed(0, 0);
    db_timer_delay_ms(TIMER_DEV, 1000);

    // Spin back
    db_motors_set_speed(70, -70);
    db_timer_delay_ms(TIMER_DEV, 2000);

    // Stop
    db_motors_set_speed(0, 0);
    db_timer_delay_ms(TIMER_DEV, 1000);

    // Blink for 2 seconds
    for (int i = 0; i < 16; i++) {
        db_gpio_toggle(&db_led1);
        db_timer_delay_ms(1, 125);
    }
    db_gpio_clear(&db_led1);

    // stop the keep alive, which will cause the robot to reboot
    keep_alive_enabled = false;

    while (1) {}
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
