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

//=========================== defines ==========================================

#define GPS_UART_MAX_BYTES (83U)  ///< max bytes in UART receive buffer 82 bytes for NMEA + 1

typedef struct {
    uint8_t timestamp[11];
    uint8_t valid;
    float   latitude;
    uint8_t latitude_N_S;
    float   longitude;
    uint8_t longitude_E_W;
    float   velocity;
    float   course;
    uint8_t datestamp[7];
    float   variation;
    uint8_t variation_E_W;
    uint8_t mode;
    uint8_t checksum;
} nmea_gprmc_t;

typedef struct {
    uint8_t      buffer[GPS_UART_MAX_BYTES];  ///< buffer where message received on UART is stored
    uint8_t      pos;                         ///< current position in the UART buffer
    nmea_gprmc_t gps_position;
} gps_vars_t;

//=========================== variables ========================================

static const gpio_t _rx_pin   = { .pin = 5, .port = 0 };
static const gpio_t _tx_pin   = { .pin = 4, .port = 0 };
static const gpio_t _en_pin   = { .pin = 31, .port = 0 };
static gps_vars_t   _gps_vars = { 0 };

//=========================== prototypes ========================================
uint8_t *strtok_new(uint8_t *string, uint8_t const *delimiter);
void     nmea_parse_gprmc_sentence(uint8_t *buffer, nmea_gprmc_t *position);

//=========================== callbacks ========================================

static void uart_callback(uint8_t byte) {
    _gps_vars.buffer[_gps_vars.pos] = byte;
    _gps_vars.pos++;
    if (byte == '\n' || _gps_vars.pos == GPS_UART_MAX_BYTES - 1) {  // parse GPS NMEA sentences
        // null-terminate the string so that we can use string.h functions
        _gps_vars.buffer[_gps_vars.pos] = NULL;

        if (memcmp(_gps_vars.buffer, "$GPRMC", 6) == 0) {
            nmea_parse_gprmc_sentence(_gps_vars.buffer, &_gps_vars.gps_position);
        }

        // reset the position for the next sentence
        _gps_vars.pos = 0;
    }
}

// from https://stackoverflow.com/questions/26522583/c-strtok-skips-second-token-or-consecutive-delimiter
// version of strtok that handles consecutive delimeters
uint8_t *strtok_new(uint8_t *string, uint8_t const *delimiter) {
    static uint8_t *source = NULL;
    uint8_t        *ret    = 0;
    uint8_t        *p;

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
void nmea_parse_gprmc_sentence(uint8_t *buffer, nmea_gprmc_t *position) {
    uint8_t *current_value;
    uint8_t  current_position;

    current_position = 0;

    printf("RECEIVED: %s", buffer);

    current_value = strtok_new(buffer, ",*\n");
    while (current_value != NULL) {
        switch (current_position) {
            case 0:  // sentence type is GPRMC
                assert(strcmp(current_value, "$GPRMC") == 0);
                break;
            case 1:  // time
                strcpy(position->timestamp, current_value);
                printf("Time: %s\n", position->timestamp);
                break;
            case 2:  // Status "A" or "V"
                if (strcmp(current_value, "A") == 0) {
                    position->valid = 1;
                } else {
                    position->valid = 0;
                }
                printf("Valid: %d\n", position->valid);
                break;
            case 3:  // Latitude
                position->latitude = atof(current_value);
                printf("Latitude: %f\n", position->latitude);
                break;
            case 4:  // Latitude N/S
                position->latitude_N_S = *current_value;
                // printf("Latitude N/S: %c\n", position->latitude_N_S);
                break;
            case 5:  // Longitude
                position->longitude = atof(current_value);
                printf("Longitude: %f\n", position->longitude);
                break;
            case 6:  // Longitude E/W
                position->longitude_E_W = *current_value;
                // printf("Longitude E/W: %c\n", position->longitude_E_W);
                break;
            case 7:  // Speed over ground in knots
                position->velocity = atof(current_value);
                printf("Speed (kn): %f\n", position->velocity);
                break;
            case 8:  // True course
                position->course = atof(current_value);
                printf("Tracke angle (deg): %f\n", position->course);
                break;
            case 9:  // Date
                strcpy(position->datestamp, current_value);
                printf("Date: %s\n", position->datestamp);
                break;
            case 10:  // Magnetic variation
                position->variation = atof(current_value);
                printf("Magnetic variation: %f\n", position->variation);
                break;
            case 11:  // Magnetic variation E/W
                position->variation_E_W = *current_value;
                // printf("Magnetic variation E/W: %c\n", position->variation_E_W);
                break;
            case 12:  // Mode indicator
                position->mode = *current_value;
                printf("Mode: %c\n", position->mode);
                break;
            case 13:
                position->checksum = strtol(current_value, NULL, 16);
                printf("Checksum: %x\n", position->checksum);
                break;
            default:
                return;
        }
        current_position++;
        current_value = strtok_new(NULL, ",*\n");
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
