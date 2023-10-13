/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application can be flashed on partition 0 (at 0x2000)
 *
 * @copyright Inria, 2023
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <nrf.h>

#include "board_config.h"
#include "gpio.h"
#include "timer.h"

int main(void) {
    puts("Booting on partition 0");

    db_timer_init();
    db_gpio_init(&db_led1, DB_GPIO_OUT);

    while (1) {
        db_gpio_toggle(&db_led1);
        db_timer_delay_ms(100);
    }
}
