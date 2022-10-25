/**
 * @file 03app_sailbot.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is the radio-controlled SailBot app.
 *
 * Load this program on your board. Now the SailBot can be remotely controlled
 * from a nearby nRF52840-DK.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>

// Include BSP packages
#include "device.h"
#include "radio.h"
#include "servos.h"
#include "gps.h"
#include "imu.h"
#include "protocol.h"
#include "timer.h"

//=========================== defines =========================================

#define SAIL_TRIM_ANGLE_UNIT_STEP (10)     // unit step increase/decrease when trimming the sails
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define TIMEOUT_CHECK_DELAY_MS    (200)    ///< 200 ms delay between packet received timeout checks
#define ADVERTISEMENT_PERIOD_MS   (500)    ///< send an advertisement every 500 ms
#define DB_BUFFER_MAX_BYTES       (64U)    ///< Max bytes in UART receive buffer

typedef struct {
    uint32_t ts_last_packet_received;  ///< Last timestamp in microseconds a control packet was received
    int8_t   sail_trim;
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    bool     imu_data_ready;
} sailbot_vars_t;

//=========================== variables =========================================

static sailbot_vars_t _sailbot_vars;

//=========================== prototypes =========================================

void        radio_callback(uint8_t *packet, uint8_t length);
void        gps_callback(nmea_gprmc_t *last_position);
void        imu_data_ready_callback(void);
static void _timeout_check(void);
static void _advertise(void);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    _sailbot_vars.sail_trim               = 0;
    _sailbot_vars.ts_last_packet_received = 0;

    // Configure Radio as a receiver
    db_radio_init(&radio_callback);  // Set the callback function.
    db_radio_set_frequency(8);       // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();            // Start receiving packets.
    db_timer_init();
    db_timer_set_periodic_ms(0, TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(1, ADVERTISEMENT_PERIOD_MS, &_advertise);

    // Init the IMU
    imu_init(&imu_data_ready_callback);

    // Configure Motors
    servos_init();

    // Configure GPS
    gps_init(gps_callback);

    // Wait for radio packets to arrive/
    while (1) {
        // processor idle until an interrupt occurs and is handled
        __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions =========================================

/**
 *  @brief Callback function to process received packets
 *
 * This function gets called each time a packet is received.
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void radio_callback(uint8_t *packet, uint8_t length) {
    uint8_t *          ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // timestamp the arrival of the packet
    _sailbot_vars.ts_last_packet_received = db_timer_ticks();

    // we filter out all packets other than MOVE_RAW command
    if (header->type != DB_PROTOCOL_CMD_MOVE_RAW) {
        return;
    }

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
        return;
    }

    // Check version is compatible
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Only move raw commands are supported
    if (header->type != DB_PROTOCOL_CMD_MOVE_RAW) {
        return;
    }

    // Read the DotBot command
    protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)(ptk_ptr + sizeof(protocol_header_t));

    if (command->right_y > 0 && ((int16_t)_sailbot_vars.sail_trim + SAIL_TRIM_ANGLE_UNIT_STEP < 127)) {
        _sailbot_vars.sail_trim += SAIL_TRIM_ANGLE_UNIT_STEP;
    } else if (command->right_y < 0 && ((int16_t)_sailbot_vars.sail_trim - SAIL_TRIM_ANGLE_UNIT_STEP > 0)) {
        _sailbot_vars.sail_trim -= SAIL_TRIM_ANGLE_UNIT_STEP;
    }

    // set the servos
    servos_set(command->left_x, _sailbot_vars.sail_trim);
}

void gps_callback(nmea_gprmc_t *last_position) {
    // TODO implement the control loop
}

void imu_data_ready_callback(void) {
    // set the flag that data is ready
    _sailbot_vars.imu_data_ready = true;
}

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks();
    if (ticks > _sailbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        // set the servos
        servos_set(0, _sailbot_vars.sail_trim);
    }
}

static void _advertise(void) {
    db_protocol_header_to_buffer(_sailbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DB_PROTOCOL_ADVERTISEMENT);
    size_t length = sizeof(protocol_header_t);
    db_radio_rx_disable();
    db_radio_tx(_sailbot_vars.radio_buffer, length);
    db_radio_rx_enable();
}
