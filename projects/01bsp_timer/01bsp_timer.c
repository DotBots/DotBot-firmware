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
#include "board.h"
#include "rgbled.h"
#include "timer.h"

//=========================== defines =========================================

//=========================== variables =========================================

static uint8_t _color_idx = 0;

//=========================== main =========================================

static void message_callback(void) {
    printf("%u - Hello from callback\n", db_timer_now());
}

static void led_callback(void) {
    _color_idx = (_color_idx + 1) % 2;
    if (_color_idx) {
        db_rgbled_set(255, 0, 0);
    }
    else {
        db_rgbled_set(0, 255, 0);
    }
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_rgbled_init();
    db_timer_init();
    db_timer_set_periodic(0, 200, &message_callback);
    db_timer_set_periodic(1, 500, &led_callback);
    while (1) {
        printf("%u - Hello dotbot\n", db_timer_now());
        db_timer_delay_s(1);
        printf("%u - Hello dotbot again\n", db_timer_now());
        db_timer_delay_ms(500);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
