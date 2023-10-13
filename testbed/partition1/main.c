/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application can be flashed on partition 1 (at 0x8000)
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

static void _btn_toggle_callback(void *ctx) {
    (void)ctx;
    db_gpio_toggle(&db_led1);
}

int main(void) {
    puts("Booting on partition 1");

    db_timer_init();

    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_gpio_set(&db_led1);
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_init_irq(&db_btn1, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, NULL);

    while (1) {
        db_gpio_toggle(&db_led2);
        db_timer_delay_s(1);
    }
}
