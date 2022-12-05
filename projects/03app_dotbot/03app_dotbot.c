/**
 * @file 03app_dotbot.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control a single dotbot remotely
 *
 * The remote control can be either a keyboard, a joystick or buttons on the gateway
 * itself
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdlib.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "motors.h"
#include "radio.h"
#include "rgbled.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_LH2_FULL_COMPUTATION   (0)
#define DB_BUFFER_MAX_BYTES       (64U)  ///< Max bytes in UART receive buffer

typedef struct {
    uint32_t ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t lh2;                                ///< LH2 device descriptor
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
} dotbot_vars_t;

//=========================== variables ========================================

///! LH2 event gpio
static const gpio_t _lh2_e_gpio = {
    .port = 0,
    .pin  = 30,
};

///! LH2 data gpio
static const gpio_t _lh2_d_gpio = {
    .port = 0,
    .pin  = 29,
};

static dotbot_vars_t _dotbot_vars;

//=========================== prototypes =======================================

static void _timeout_check(void);
static void _advertise(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    _dotbot_vars.ts_last_packet_received = db_timer_ticks();
    do {
        uint8_t *          ptk_ptr = pkt;
        protocol_header_t *header  = (protocol_header_t *)ptk_ptr;
        // Check destination address matches
        if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
            break;
        }

        // Check version is supported
        if (header->version != DB_FIRMWARE_VERSION) {
            break;
        }

        // Check application is compatible
        if (header->application != DotBot) {
            break;
        }

        uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
        // parse received packet and update the motors' speeds
        switch (header->type) {
            case DB_PROTOCOL_CMD_MOVE_RAW:
            {
                protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
                int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
                int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
                db_motors_set_speed(left, right);
            } break;
            case DB_PROTOCOL_CMD_RGB_LED:
            {
                protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
                db_rgbled_set(command->r, command->g, command->b);
            } break;
            default:
                break;
        }
    } while (0);
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_rgbled_init();
    db_motors_init();
    db_radio_init(&radio_callback);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();       // Start receiving packets.
    db_timer_init();
    db_timer_set_periodic_ms(0, 200, &_timeout_check);
    db_timer_set_periodic_ms(1, 500, &_advertise);
    db_lh2_init(&_dotbot_vars.lh2, &_lh2_d_gpio, &_lh2_e_gpio);
    db_lh2_start(&_dotbot_vars.lh2);

    while (1) {
        db_lh2_process_raw_data(&_dotbot_vars.lh2);
        if (_dotbot_vars.lh2.state == DB_LH2_RAW_DATA_READY) {
            db_lh2_stop(&_dotbot_vars.lh2);
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_LH2_RAW_DATA);
            memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), _dotbot_vars.lh2.raw_data, sizeof(db_lh2_raw_data_t) * LH2_LOCATIONS_COUNT);
            size_t length = sizeof(protocol_header_t) + sizeof(db_lh2_raw_data_t) * LH2_LOCATIONS_COUNT;
            db_radio_rx_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            db_radio_rx_enable();
            if (DB_LH2_FULL_COMPUTATION) {
                // the location function has to be running all the time
                db_lh2_process_location(&_dotbot_vars.lh2);

                // Reset the LH2 driver if a packet is ready. At this point, locations
                // can be read from lh2.results array
                if (_dotbot_vars.lh2.state == DB_LH2_LOCATION_READY) {
                    __NOP();  // Add this no-op to allow setting a breakpoint here
                }
            }
            db_lh2_start(&_dotbot_vars.lh2);
        }
        db_timer_delay_ms(50);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== private functions ================================

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks();
    if (ticks > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        db_motors_set_speed(0, 0);
    }
}

static void _advertise(void) {
    db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
    size_t length = sizeof(protocol_header_t);
    db_radio_rx_disable();
    db_radio_tx(_dotbot_vars.radio_buffer, length);
    db_radio_rx_enable();
}
