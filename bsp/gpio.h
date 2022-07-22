#ifndef __GPIO_H
#define __GPIO_H

/**
 * @file gpio.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "gpio" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdint.h>

//=========================== defines ==========================================

/**
 * Enable a pin as output, must be called once before using the other macros below
 */
#define DB_GPIO_ENABLE(port, pin)   NRF_P##port->DIRSET = (1 << pin)
#define DB_GPIO_ON(port, pin)       NRF_P##port->OUTSET = (1 << pin)    ///< Turn on the pin
#define DB_GPIO_OFF(port, pin)      NRF_P##port->OUTCLR = (1 << pin)    ///< Turn off the pin
#define DB_GPIO_TOGGLE(port, pin)   NRF_P##port->OUT   ^= (1 << pin)    ///< Toggle the pin

typedef struct {
    uint8_t port;   ///< Port number of the GPIO
    uint8_t pin;    ///< Pin number of the GPIO
} gpio_t;

//=========================== variables ========================================

#ifdef NRF_P1
static NRF_GPIO_Type *nrf_port[2] = { NRF_P0, NRF_P1 };
#else
static NRF_GPIO_Type *nrf_port[2] = { NRF_P0, NULL };
#endif

#endif
