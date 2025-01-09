/**
 * @file
 * @defgroup swarmit_timer    Timer application on top of SwarmIT
 * @ingroup swarmit
 * @brief   This application blinks two LEDs using the timer API
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board_config.h"
#include "gpio.h"
#include "timer.h"

//=========================== swarmit ==========================================

void swarmit_reload_wdt0(void);

//=========================== defines ==========================================

#define TIMER_DEV0 0

//=========================== variables ========================================

static const gpio_t led1 = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };
static const gpio_t led2 = { .port = DB_LED2_PORT, .pin = DB_LED2_PIN };

//=========================== callbacks ========================================

static void _leds_callback(void) {
    swarmit_reload_wdt0();
    db_gpio_toggle(&led1);
    db_gpio_toggle(&led2);
}

//=========================== main =============================================

int main(void) {
    db_gpio_init(&led1, DB_GPIO_OUT);
    db_gpio_init(&led2, DB_GPIO_OUT);
    db_gpio_set(&led2);
    db_timer_init(TIMER_DEV0);
    db_timer_set_periodic_ms(TIMER_DEV0, 0, 500, &_leds_callback);

    while (1) {
        __WFE();
    }
}
