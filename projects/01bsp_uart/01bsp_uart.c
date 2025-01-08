/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the UART API.
 *
 * @copyright Inria, 2022
 *
 */

#include <stdint.h>
#include <nrf.h>
#include "board.h"
#include "board_config.h"
#include "uart.h"

//=========================== defines ==========================================

#define DB_UART_MAX_BYTES (64U)       ///< max bytes in UART receive buffer
#define DB_UART_BAUDRATE  (1000000U)  ///< UART baudrate

typedef struct {
    uint8_t buffer[DB_UART_MAX_BYTES];  ///< buffer where message received on UART is stored
    uint8_t pos;                        ///< current position in the UART buffer
} uart_vars_t;

//=========================== variables ========================================

static uart_vars_t _uart_vars = { 0 };

//=========================== callbacks ========================================

static void uart_callback(uint8_t byte) {
    _uart_vars.buffer[_uart_vars.pos] = byte;
    _uart_vars.pos++;
    if (byte == '\n' || _uart_vars.pos == DB_UART_MAX_BYTES - 1) {
        db_uart_write(0, _uart_vars.buffer, _uart_vars.pos);
        _uart_vars.pos = 0;
    }
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_uart_init(0, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &uart_callback);

    while (1) {
        __WFE();
    }
}
