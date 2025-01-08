/**
 * @file
 * @defgroup swarmit_rgbled    RGBLed application on top of SwarmIT
 * @ingroup swarmit
 * @brief   This application uses the RGB LED API
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "board_config.h"

//=========================== swarmit ==========================================

void swarmit_reload_wdt0(void);

//=========================== defines ==========================================

#define TIMER_DEV        (0)
#define RGB_LED_DELAY_MS (200U)

//=========================== variables ========================================

static const db_rgbled_pwm_conf_t rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};

//=========================== main =============================================

int main(void) {
    db_board_init();

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, 500, &swarmit_reload_wdt0);

    db_rgbled_pwm_init(&rgbled_pwm_conf);

    /* Change RGB colors in a loop */
    while (1) {
        db_rgbled_pwm_set_color(255, 0, 0);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
        db_rgbled_pwm_set_color(0, 255, 0);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
        db_rgbled_pwm_set_color(0, 0, 255);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
        db_rgbled_pwm_set_color(255, 255, 0);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
        db_rgbled_pwm_set_color(0, 255, 255);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
        db_rgbled_pwm_set_color(255, 0, 255);
        db_timer_delay_ms(TIMER_DEV, RGB_LED_DELAY_MS);
    }
}
