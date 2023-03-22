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
 * @copyright Inria, 2022-2023
 */

#include <nrf.h>
#include <stdint.h>

//=========================== defines ==========================================

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define NRF_P0            NRF_P0_S
#define NRF_P1            NRF_P1_S
#define NRF_GPIOTE        NRF_GPIOTE0_S
#define GPIOTE_IRQn       GPIOTE0_IRQn
#define GPIOTE_IRQHandler GPIOTE0_IRQHandler
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define NRF_P0     NRF_P0_NS
#define NRF_P1     NRF_P1_NS
#define NRF_GPIOTE NRF_GPIOTE_NS
#endif

/**
 * Enable a pin as output, must be called once before using the other macros below
 */
#define DB_GPIO_ENABLE(port, pin) NRF_P##port->DIRSET = (1 << pin)  ///< Enable a pin as output
#define DB_GPIO_ON(port, pin)     NRF_P##port->OUTSET = (1 << pin)  ///< Turn on the pin
#define DB_GPIO_OFF(port, pin)    NRF_P##port->OUTCLR = (1 << pin)  ///< Turn off the pin
#define DB_GPIO_TOGGLE(port, pin) NRF_P##port->OUT ^= (1 << pin)    ///< Toggle the pin

typedef void (*gpio_cb_t)(void *ctx);  ///< Callback function prototype, it is called on each gpio interrupt

typedef enum {
    DB_GPIO_OUT,    ///< Floating output
    DB_GPIO_IN,     ///< Floating input
    DB_GPIO_IN_PU,  ///< Pull up input
    DB_GPIO_IN_PD,  ///< Pull down input
} gpio_mode_t;

typedef enum {
    DB_GPIO_IRQ_EDGE_RISING  = GPIOTE_CONFIG_POLARITY_LoToHi,  ///< Rising edge
    DB_GPIO_IRQ_EDGE_FALLING = GPIOTE_CONFIG_POLARITY_HiToLo,  ///< Falling edge
    DB_GPIO_IRQ_EDGE_BOTH    = GPIOTE_CONFIG_POLARITY_Toggle,  ///< Both falling and rising edges
} gpio_irq_edge_t;

typedef struct {
    uint8_t port;  ///< Port number of the GPIO
    uint8_t pin;   ///< Pin number of the GPIO
} gpio_t;

//=========================== variables ========================================

static NRF_GPIO_Type *nrf_port[2] = { NRF_P0, NRF_P1 };

//============================ public ==========================================

/**
 * @brief   Initialize a GPIO
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 * @param[in]   mode            GPIO mode (output, input, etc)
 */
void db_gpio_init(const gpio_t *gpio, gpio_mode_t mode);

/**
 * @brief   Initialize a GPIO with IRQ
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 * @param[in]   mode            GPIO mode, should one flavor of input
 * @param[in]   edge            Type of edge to trigger the IRQ (Falling, Rising or both)
 * @param[in]   callback        Function pointer that is called from gpio ISR
 * @param[in]   ctx             Pointer to some context passed as parameter to the callback
 */
void db_gpio_init_irq(const gpio_t *gpio, gpio_mode_t mode, gpio_irq_edge_t edge, gpio_cb_t callback, void *ctx);

/**
 * @brief   Set the GPIO output high
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 */
void db_gpio_set(const gpio_t *gpio);

/**
 * @brief   Set the GPIO output low
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 */
void db_gpio_clear(const gpio_t *gpio);

/**
 * @brief   Toggle the GPIO output
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 */
void db_gpio_toggle(const gpio_t *gpio);

/**
 * @brief   Read the value of a GPIO
 *
 * @param[in]   gpio            Pointer to the GPIO descriptor
 * @return                      GPIO level (0 or 1)
 */
uint8_t db_gpio_read(const gpio_t *gpio);

#endif
