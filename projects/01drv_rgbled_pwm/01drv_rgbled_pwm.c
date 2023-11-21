/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to interface with the RGB LED in the DotBot shield.
 *
 * Load this program on your board. The LEDs should start blinking different colors.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "board_config.h"

//=========================== variables ========================================

#ifdef DB_RGB_LED_PWM_RED_PORT
static const db_rgbled_pwm_conf_t rgbled_pwm_conf = {
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

//=========================== main =============================================

int main(void) {
    db_board_init();
    db_timer_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&rgbled_pwm_conf);
#endif

    /* Change RGB colors in a loop */
    while (1) {
#ifdef DB_RGB_LED_PWM_RED_PORT
        db_rgbled_pwm_set_color(255, 0, 0);
        db_timer_delay_ms(100);
        db_rgbled_pwm_set_color(0, 255, 0);
        db_timer_delay_ms(100);
        db_rgbled_pwm_set_color(0, 0, 255);
        db_timer_delay_ms(100);
        db_rgbled_pwm_set_color(255, 255, 0);
        db_timer_delay_ms(100);
        db_rgbled_pwm_set_color(0, 255, 255);
        db_timer_delay_ms(100);
        db_rgbled_pwm_set_color(255, 0, 255);
#else
        puts("RGB LED pwm not supported!");
#endif
        db_timer_delay_ms(100);
    }
}
