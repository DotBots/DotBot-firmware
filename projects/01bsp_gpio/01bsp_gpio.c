/**
 * @file 01bsp_gpio.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the GPIO api.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"

//=========================== variables ========================================

static const gpio_t led1 = { .port = 0, .pin = 13 };
static const gpio_t led2 = { .port = 0, .pin = 14 };
static const gpio_t led3 = { .port = 0, .pin = 15 };
static const gpio_t btn1 = { .port = 0, .pin = 11 };
static const gpio_t btn2 = { .port = 0, .pin = 12 };
static const gpio_t btn3 = { .port = 0, .pin = 24 };
static const gpio_t btn4 = { .port = 0, .pin = 25 };

//=========================== callbacks ========================================

static void _btn_toggle_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_toggle(gpio);
}

static void _btn_clear_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_clear(gpio);
}

static void _btn_set_callback(void *ctx) {
    const gpio_t *gpio = (const gpio_t *)ctx;
    db_gpio_set(gpio);
}

//=========================== main =============================================

int main(void) {
    db_gpio_init(&led1, DB_GPIO_OUT);
    db_gpio_set(&led1);

    db_gpio_init(&led2, DB_GPIO_OUT);
    db_gpio_set(&led2);

    db_gpio_init(&led3, DB_GPIO_OUT);
    db_gpio_set(&led3);

    db_gpio_init_irq(&btn1, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, (void *)&led1);
    db_gpio_init_irq(&btn2, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, (void *)&led2);
    db_gpio_init_irq(&btn3, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_clear_callback, (void *)&led3);
    db_gpio_init_irq(&btn4, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_set_callback, (void *)&led3);

    while (1) {
        __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
