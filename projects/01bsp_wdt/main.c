/**
 * @file
 * @ingroup samples_wdt
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Sample application using the Watchdog timer driver
 *
 * @copyright Inria, 2024
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <nrf.h>
#include "board_config.h"
#include "gpio.h"
#include "timer_hf.h"
#include "wdt.h"

#define WDT_RELOAD_COUNT 5

#if defined(NRF5340_XXAA) && (NRF_APPLICATION)
#define NRF_RESET NRF_RESET_S
#else
#define NRF_RESET NRF_RESET_NS
#endif

static void _wdt_cb(void *ctx) {
    (void)ctx;
    db_gpio_toggle(&db_led1);
}

int main(void) {
    printf("Starting watchdog timer\n");
#if defined(NRF5340_XXAA)
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_set(&db_led2);
    uint32_t resetreas   = NRF_RESET->RESETREAS;
    NRF_RESET->RESETREAS = NRF_RESET->RESETREAS;
    if (resetreas & RESET_RESETREAS_DOG0_Detected << RESET_RESETREAS_DOG0_Pos) {
        db_gpio_clear(&db_led2);
    }
#endif

    db_timer_hf_init(0);
    db_timer_hf_delay_s(0, 1);
#if defined(NRF5340_XXAA)
    db_gpio_set(&db_led2);
#endif
    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_wdt_init(2, _wdt_cb, NULL);
    db_wdt_start();

    uint8_t reloads = WDT_RELOAD_COUNT;
    while (reloads--) {
        printf("Reloading... %d\n", reloads + 1);
        db_wdt_reload();
        db_timer_hf_delay_s(0, 1);
    }
    puts("Stop reload");

    while (1) {
        __WFE();
    }
}
