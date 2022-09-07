/**
 * @file gps.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for reading the GPS on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"
#include "uart.h"

//=========================== defines ==========================================

#define GPS_UART_MAX_BYTES (64U)  ///< max bytes in UART receive buffer

typedef struct {
    uint8_t buffer[GPS_UART_MAX_BYTES];  ///< buffer where message received on UART is stored
    uint8_t pos;                        ///< current position in the UART buffer
} gps_vars_t;

//=========================== variables ========================================

static const gpio_t _rx_pin    = { .pin = 5, .port = 0 };
static const gpio_t _tx_pin    = { .pin = 4, .port = 0 };
static gps_vars_t  _gps_vars = { 0 };

//=========================== callbacks ========================================

static void uart_callback(uint8_t byte) {
    _gps_vars.buffer[_gps_vars.pos] = byte;
    _gps_vars.pos++;
    if (byte == '\n' || _gps_vars.pos == GPS_UART_MAX_BYTES - 1) {
        // TODO parse GPS data
        _gps_vars.pos = 0;
    }
}

/**
 *  @brief Initialization routine of the GPS module.
 */
 void gps_init() {
    // Turn ON the GPS module
    NRF_P0->DIRSET = 1 << 31;  // set pin as output
    NRF_P0->OUTSET = 1 << 31;  // set pin HIGH

    // configure UART at 9600 bauds
    db_uart_init(&_rx_pin, &_tx_pin, 9600, &uart_callback);
 }

 void gps_last_known_position(void) {
    return;
 }