#ifndef __UART_H
#define __UART_H

/**
 * @defgroup    bsp_uart    UART
 * @ingroup     bsp
 * @brief       Control the UART peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"

//=========================== defines ==========================================

typedef uint8_t uart_t;  ///< UART peripheral index

typedef void (*uart_rx_cb_t)(uint8_t data);  ///< Callback function prototype, it is called on each byte received

//=========================== public ===========================================

/**
 * @brief Initialize the UART interface
 *
 * @param[in] uart      UART peripheral to use
 * @param[in] rx_pin    pointer to RX pin
 * @param[in] tx_pin    pointer to TX pin
 * @param[in] baudrate  Baudrate in bauds
 * @param[in] callback  callback function called on each received byte
 */
void db_uart_init(uart_t uart, const gpio_t *rx_pin, const gpio_t *tx_pin, uint32_t baudrate, uart_rx_cb_t callback);

/**
 * @brief Write bytes to the UART
 *
 * @param[in] uart      UART peripheral to use
 * @param[in] buffer    pointer to the buffer to write to UART
 * @param[in] length    number of bytes of the buffer to write
 */
void db_uart_write(uart_t uart, uint8_t *buffer, size_t length);

#endif
