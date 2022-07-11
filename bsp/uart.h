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

#define DB_UART_MAX_BYTES   (32)

typedef void(*uart_rx_cb_t)(uint8_t data);

//=========================== public ===========================================

void db_uart_init(const gpio_t *rx_pin, const gpio_t *tx_pin, uint32_t baudrate, uart_rx_cb_t callback);
void db_uart_write(uint8_t *buffer, size_t length);

#endif
