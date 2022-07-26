#ifndef __UART_H
#define __UART_H

/**
 * @file uart.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "uart" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"

//=========================== defines ==========================================

typedef void (*uart_rx_cb_t)(uint8_t data);  ///< Callback function prototype, it is called on each byte received

//=========================== public ===========================================

/**
 * @brief Initialize the UART interface
 *
 * @param[in] rx_pin    pointer to RX pin
 * @param[in] tx_pin    pointer to TX pin
 * @param[in] callback  callback function called on each received byte
 */
void db_uart_init(const gpio_t *rx_pin, const gpio_t *tx_pin, uart_rx_cb_t callback);

/**
 * @brief Write bytes to the UART
 *
 * @param[in] buffer    pointer to the buffer to write to UART
 * @param[in] length    number of bytes of the buffer to write
 */
void db_uart_write(uint8_t *buffer, size_t length);

#endif
