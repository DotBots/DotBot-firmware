/**
 * @file 01bsp_timer.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to add delays and periodic callbacks in an application.
 *
 * Load this program on your board:
 *  - the LED will toggle to red and green every 500ms
 *  - a message is printed every 200ms in the debug terminal
 *  - in a loop, other messages are printed with some delays (1s and 500ms)
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "timer.h"

//=========================== defines =========================================

//=========================== variables =========================================

#if defined(NRF5340_XXAA)
static const gpio_t led1 = { .port = 0, .pin = 28 };
#else
static const gpio_t led1 = { .port = 0, .pin = 13 };
#endif

//=========================== main =========================================

static void message_callback(void) {
    printf("Hello from callback\n");
}

static void message_one_shot_callback(void) {
    printf("Hello from one shot callback\n");
}

static void led_callback(void) {
    db_gpio_toggle(&led1);
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_gpio_init(&led1, DB_GPIO_OUT);
    db_gpio_set(&led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 2000, &message_callback);
    db_timer_set_periodic_ms(1, 500, &led_callback);
    db_timer_set_oneshot_ms(2, 1000, &message_one_shot_callback);
    while (1) {
        printf("Hello dotbot\n");
        db_timer_delay_ms(500);
        printf("Hello dotbot again\n");
        db_timer_delay_s(1);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
