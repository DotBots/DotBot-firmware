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
#include <string.h>
#include "assert.h"
#include "gpio.h"
#include "uart.h"
#include "gps.h"

//=========================== defines ==========================================

#define GPS_UART_MAX_BYTES (83U)  ///< max bytes in UART receive buffer 82 bytes for NMEA + 1

typedef struct {
    uint8_t      buffer[GPS_UART_MAX_BYTES];  ///< buffer where message received on UART is stored
    uint8_t      pos;                         ///< current position in the UART buffer
    nmea_gprmc_t gps_position;                ///< last parsed RMC sentence containing the position
    gps_rx_cb_t  callback;                    ///< GPS callback function configured by the application
} gps_vars_t;

//=========================== variables ========================================

static const gpio_t _rx_pin   = { .pin = 5, .port = 0 };
static const gpio_t _tx_pin   = { .pin = 4, .port = 0 };
static const gpio_t _en_pin   = { .pin = 31, .port = 0 };
static gps_vars_t   _gps_vars = { 0 };

//=========================== prototypes ========================================

uint8_t *strtok_new(uint8_t *string, uint8_t const *delimiter);
int      nmea_parse_gprmc_sentence(uint8_t *buffer, nmea_gprmc_t *position);
uint8_t  nmea_calculate_checksum(uint8_t *buffer);

//=========================== callbacks ========================================

static void uart_callback(uint8_t byte) {
    int parsing_error;

    // initialize to error
    parsing_error = -1;

    _gps_vars.buffer[_gps_vars.pos] = byte;
    _gps_vars.pos++;
    if (byte == '\n' || _gps_vars.pos == GPS_UART_MAX_BYTES - 1) {  // parse GPS NMEA sentences
        // null-terminate the string so that we can use string.h functions
        _gps_vars.buffer[_gps_vars.pos] = NULL;

        // if not $GPRMC return
        if (memcmp(_gps_vars.buffer, "$GPRMC", 6) != 0) {
            // reset the position for the next sentence
            _gps_vars.pos = 0;
            return;
        }

        // parse the $GPRMC sentence
        parsing_error = nmea_parse_gprmc_sentence(_gps_vars.buffer, &_gps_vars.gps_position);

        // invoke the callback if parsing succeeded
        if (_gps_vars.callback != NULL && parsing_error == 0) {
            _gps_vars.callback(&_gps_vars.gps_position);
        }

        // reset the position for the next sentence
        _gps_vars.pos = 0;
    }
}

// from https://stackoverflow.com/questions/26522583/c-strtok-skips-second-token-or-consecutive-delimiter
// version of strtok that handles consecutive delimeters
uint8_t *strtok_new(uint8_t *string, uint8_t const *delimiter) {
    static uint8_t *source = NULL;
    uint8_t *       ret    = 0;
    uint8_t *       p;

    if (string != NULL) {
        source = string;
    }

    if (source == NULL) {
        return NULL;
    }

    if ((p = strpbrk(source, delimiter)) != NULL) {
        *p     = 0;
        ret    = source;
        source = ++p;
    }

    return ret;
}

// buffer is a NULL-terminated string
int nmea_parse_gprmc_sentence(uint8_t *buffer, nmea_gprmc_t *position) {
    uint8_t  coordinate_degrees[4];
    uint8_t *current_value;
    uint8_t  current_position;

    current_position = 0;

    current_value = strtok_new(buffer, ",*\n");
    while (current_value != NULL) {
        switch (current_position) {
            case 0:  // sentence type is GPRMC
                assert(strcmp(current_value, "$GPRMC") == 0);
                break;
            case 1:  // time
                strcpy(position->timestamp, current_value);
                break;
            case 2:  // Status "A" or "V"
                if (strcmp(current_value, "A") == 0) {
                    position->valid = 1;
                } else {
                    position->valid = 0;
                }
                break;
            case 3:  // Latitude
                position->latitude = 0;
                if (strlen(current_value) > 2) {
                    coordinate_degrees[0] = current_value[0];
                    coordinate_degrees[1] = current_value[1];
                    coordinate_degrees[2] = NULL;
                    position->latitude += atoi(coordinate_degrees);
                    position->latitude += atof(&current_value[2]) / 60.0;
                }
                break;
            case 4:  // Latitude N/S
                position->latitude_N_S = *current_value;
                break;
            case 5:  // Longitude
                position->longitude = 0;
                if (strlen(current_value) > 3) {
                    coordinate_degrees[0] = current_value[0];
                    coordinate_degrees[1] = current_value[1];
                    coordinate_degrees[2] = current_value[2];
                    coordinate_degrees[3] = NULL;
                    position->longitude += atoi(coordinate_degrees);
                    position->longitude += atof(&current_value[3]) / 60.0;
                }
                break;
            case 6:  // Longitude E/W
                position->longitude_E_W = *current_value;
                break;
            case 7:  // Speed over ground in knots
                position->velocity = atof(current_value);
                break;
            case 8:  // True course
                position->course = atof(current_value);
                break;
            case 9:  // Date
                strcpy(position->datestamp, current_value);
                break;
            case 10:  // Magnetic variation
                position->variation = atof(current_value);
                break;
            case 11:  // Magnetic variation E/W
                position->variation_E_W = *current_value;
                break;
            case 12:  // Mode indicator
                position->mode = *current_value;
                break;
            case 13:
                // TODO verify checksum
                break;
            default:
                return -1;
        }
        current_position++;
        current_value = strtok_new(NULL, ",*\n");
    }
    return 0;
}

uint8_t nmea_calculate_checksum(uint8_t *buffer) {
    uint8_t  checksum;
    uint16_t len;
    uint16_t i;

    checksum = 0;
    len      = strlen(buffer);

    if (buffer[0] != '$') {
        return 0;
    }

    for (i = 1; i < len - 5; i++) {
        checksum ^= buffer[i];
    }

    return checksum;
}

void gps_init(gps_rx_cb_t callback) {
    // configure the module to output rate of 10 Hz
    uint8_t nmea_cmd_set_10hz_data_rate[] = "$PMTK220,100*2F\r\n";
    // configure the module at 38400 bauds
    uint8_t nmea_cmd_set_baud_rate_38400[] = "$PMTK251,38400*27\r\n";
    // enable only GPRMC sentences
    uint8_t nmea_cmd_set_nmea_output[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";

    // Turn ON the GPS module
    NRF_P0->DIRSET = 1 << _en_pin.pin;  // set pin as output
    NRF_P0->OUTSET = 1 << _en_pin.pin;  // set pin HIGH

    // configure UART at 9600 bauds
    db_uart_init(&_rx_pin, &_tx_pin, 9600, &uart_callback);
    for (int i = 0; i < 100000; i++) {
        ;
    }

    // command the module to increase the baud rate to 38400
    db_uart_write(nmea_cmd_set_baud_rate_38400, 19);

    // reinit myself at 38400 bauds
    db_uart_init(&_rx_pin, &_tx_pin, 38400, &uart_callback);
    for (int i = 0; i < 100000; i++) {
        ;
    }

    db_uart_write(nmea_cmd_set_nmea_output, 51);
    for (int i = 0; i < 100000; i++) {
        ;
    }

    // command to module to use 10hz output data rate
    db_uart_write(nmea_cmd_set_10hz_data_rate, 17);

    _gps_vars.callback = callback;
}

nmea_gprmc_t *gps_last_known_position(void) {
    return &_gps_vars.gps_position;
}
