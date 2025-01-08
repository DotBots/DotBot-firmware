/**
 * @file
 * @ingroup samples_bsp
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
#include "board_config.h"
#include "gpio.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV0 0
#define TIMER_DEV1 1

//=========================== variables ========================================

static const gpio_t led1 = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };

//=========================== callbacks ========================================

static void message_callback(void) {
    printf("Hello from callback\n");
}

static void message_one_shot_callback(void) {
    printf("Hello from one shot callback\n");
}

static void led_callback(void) {
    db_gpio_toggle(&led1);
}

//=========================== main =============================================

int main(void) {
    db_gpio_init(&led1, DB_GPIO_OUT);
    db_gpio_set(&led1);
    db_timer_init(TIMER_DEV0);
    db_timer_init(TIMER_DEV1);
    db_timer_set_periodic_ms(TIMER_DEV1, 0, 2000, &message_callback);
    db_timer_set_periodic_ms(TIMER_DEV1, 1, 500, &led_callback);
    db_timer_set_oneshot_ms(TIMER_DEV1, 2, 1000, &message_one_shot_callback);
    while (1) {
        printf("Hello dotbot\n");
        db_timer_delay_ms(TIMER_DEV0, 500);
        printf("Hello dotbot again\n");
        db_timer_delay_s(TIMER_DEV0, 1);
    }
}
