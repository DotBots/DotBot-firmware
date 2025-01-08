/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the GPIO api.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include "board_config.h"
#include "gpio.h"

//=========================== callbacks ========================================

static void _btn_toggle_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_toggle(gpio);
}

#if defined(DB_BTN3_PORT)
static void _btn_clear_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_clear(gpio);
}
#endif

#if defined(DB_BTN4_PORT)
static void _btn_set_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_set(gpio);
}
#endif

//=========================== main =============================================

int main(void) {

#if defined(DB_LED1_PORT)
    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_gpio_set(&db_led1);
#endif

#if defined(DB_LED2_PORT)
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_set(&db_led2);
#endif

#if defined(DB_LED3_PORT)
    db_gpio_init(&db_led3, DB_GPIO_OUT);
    db_gpio_set(&db_led3);
#endif

#if defined(DB_BTN1_PORT)
    db_gpio_init_irq(&db_btn1, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, (void *)&db_led1);
#endif

#if defined(DB_BTN2_PORT)
    db_gpio_init_irq(&db_btn2, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, (void *)&db_led2);
#endif

#if defined(DB_BTN3_PORT)
    db_gpio_init_irq(&db_btn3, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_clear_callback, (void *)&db_led3);
#endif

#if defined(DB_BTN4_PORT)
    db_gpio_init_irq(&db_btn4, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_set_callback, (void *)&db_led3);
#endif

    while (1) {
        __WFE();
    }
}
