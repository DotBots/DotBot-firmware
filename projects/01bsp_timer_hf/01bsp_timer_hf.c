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
#include "timer_hf.h"

//=========================== defines =========================================

//=========================== variables =========================================

static uint8_t _color_idx = 0;

//=========================== main =========================================

static void message_callback(void) {
    printf("Hello from callback\n");
}

static void message_one_shot_callback(void) {
    printf("Hello from one shot callback\n");
}

static void led_callback(void) {
    _color_idx = (_color_idx + 1) % 2;
    if (_color_idx) {
        db_rgbled_set(255, 0, 0);
    } else {
        db_rgbled_set(0, 255, 0);
    }
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_rgbled_init();
    db_timer_hf_init();
    db_timer_hf_set_periodic_us(0, 2000000, &message_callback);
    db_timer_hf_set_periodic_us(1, 500000, &led_callback);
    db_timer_hf_set_oneshot_ms(2, 1000, &message_one_shot_callback);
    while (1) {
        printf("Hello dotbot\n");
        db_timer_hf_delay_ms(500);
        printf("Hello dotbot again\n");
        db_timer_hf_delay_s(1);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
