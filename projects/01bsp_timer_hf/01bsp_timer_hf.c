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
#include "timer_hf.h"

//=========================== variables ====================================

#if defined(DB_LED2_PORT)
static const gpio_t led = { .port = DB_LED2_PORT, .pin = DB_LED2_PIN };
#else
static const gpio_t led = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };
#endif

//=========================== callbacks ====================================

static void message_callback(void) {
    printf("Hello from callback\n");
}

static void message_one_shot_callback(void) {
    printf("Hello from one shot callback\n");
}

static void led_callback(void) {
    db_gpio_toggle(&led);
}

//=========================== main =========================================

int main(void) {
    db_gpio_init(&led, DB_GPIO_OUT);
    db_gpio_set(&led);
    db_timer_hf_init(0);
    db_timer_hf_init(1);
    db_timer_hf_init(2);
    db_timer_hf_set_periodic_us(0, 0, 2000000, &message_callback);
    db_timer_hf_set_periodic_us(1, 1, 500000, &led_callback);
    db_timer_hf_set_oneshot_ms(2, 2, 1000, &message_one_shot_callback);
    while (1) {
        printf("Hello dotbot\n");
        db_timer_hf_delay_ms(0, 500);
        printf("Hello dotbot again\n");
        db_timer_hf_delay_s(0, 1);
    }
}
