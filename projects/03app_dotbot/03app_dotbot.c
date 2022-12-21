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
#include <math.h>
// Include BSP headers
#include "board.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "motors.h"
#include "pid.h"
#include "radio.h"
#include "rgbled.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMEOUT_CHECK_DELAY_TICKS      (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_LH2_FULL_COMPUTATION        (0)
#define DB_BUFFER_MAX_BYTES            (64U)   ///< Max bytes in UART receive buffer
#define DB_DIRECTION_THRESHOLD         (0.01)  ///< Threshold to update the direction
#define DB_WAYPOINT_DISTANCE_THRESHOLD (0.05)  ///< Distance threshold towards a target waypoint
#define DB_PID_SAMPLE_TIME_MS          (100)   ///< PID sample time in milliseconds
#define DB_LINEAR_SPEED                (80)    ///< Linear speed in autonomous control is constant

typedef struct {
    float x;
    float y;
    float z;
} dotbot_lh2_location_t;

typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t                 lh2;                                ///< LH2 device descriptor
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    dotbot_lh2_location_t    last_location;                      ///< Last computed LH2 location received
    int16_t                  direction;                          ///< Current direction of the DotBot (angle in °)
    protocol_control_mode_t  control_mode;                       ///< Remote control mode
    protocol_lh2_waypoints_t waypoints;                          ///< List of waypoints
    uint8_t                  next_waypoint_idx;                  ///< Index of next waypoint to reach
    pid_t                    pid_angular;                        ///< PID used to compute the angular speed
} dotbot_vars_t;

//=========================== variables ========================================

///! PID gains
static const pid_gains_t _pid_params = {
    .kp = 5,
    .ki = 2,
    .kd = 0,
};

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
static void _compute_direction(const dotbot_lh2_location_t *location, int16_t *direction);
static void _update_control_loop(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_ticks();
    do {
        uint8_t           *ptk_ptr = pkt;
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
            case DB_PROTOCOL_LH2_LOCATION:
            {
                const protocol_lh2_location_t *location     = (const protocol_lh2_location_t *)cmd_ptr;
                dotbot_lh2_location_t          new_location = {
                             .x = (float)location->x / 1e6,
                             .y = (float)location->y / 1e6,
                             .z = (float)location->z / 1e6,
                };
                _compute_direction(&new_location, &_dotbot_vars.direction);
                _dotbot_vars.last_location.x = new_location.x;
                _dotbot_vars.last_location.y = new_location.y;
                _dotbot_vars.last_location.z = new_location.z;
                if (_dotbot_vars.control_mode == ControlAuto) {
                    _update_control_loop();
                    __NOP();
                }
            } break;
            case DB_PROTOCOL_CONTROL_MODE:
                _dotbot_vars.control_mode = (protocol_control_mode_t)(*cmd_ptr);
                if (_dotbot_vars.control_mode == ControlAuto) {
                    db_pid_set_mode(&_dotbot_vars.pid_angular, DB_PID_MODE_AUTO);
                } else {
                    db_pid_set_mode(&_dotbot_vars.pid_angular, DB_PID_MODE_MANUAL);
                }
                break;
            case DB_PROTOCOL_LH2_WAYPOINTS:
            {
                _dotbot_vars.control_mode           = ControlManual;
                protocol_lh2_waypoints_t *waypoints = (protocol_lh2_waypoints_t *)cmd_ptr;
                _dotbot_vars.waypoints.length       = waypoints->length;
                memcpy(_dotbot_vars.waypoints.points, waypoints->points, waypoints->length * sizeof(protocol_lh2_location_t));
                _dotbot_vars.next_waypoint_idx = 0;
                _dotbot_vars.control_mode      = ControlAuto;
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
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();       // Start receiving packets.
    db_timer_init();
    db_timer_set_periodic_ms(0, 200, &_timeout_check);
    db_timer_set_periodic_ms(1, 500, &_advertise);
    db_lh2_init(&_dotbot_vars.lh2, &_lh2_d_gpio, &_lh2_e_gpio);
    db_lh2_start(&_dotbot_vars.lh2);

    // Initialize the pids
    db_pid_init(&_dotbot_vars.pid_angular, 0.0, 0.0,
                _pid_params.kp, _pid_params.ki, _pid_params.kd,
                -75.0, 75.0, DB_PID_SAMPLE_TIME_MS, DB_PID_MODE_MANUAL, DB_PID_DIRECTION_DIRECT);

    // Set an invalid heading since the value is unknown on startup.
    _dotbot_vars.direction = (int16_t)0xffff;

    while (1) {
        db_lh2_process_raw_data(&_dotbot_vars.lh2);
        if (_dotbot_vars.lh2.state == DB_LH2_RAW_DATA_READY) {
            db_lh2_stop(&_dotbot_vars.lh2);
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_DOTBOT_DATA);
            memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), &_dotbot_vars.direction, sizeof(int16_t));
            memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t) + sizeof(int16_t), _dotbot_vars.lh2.raw_data, sizeof(db_lh2_raw_data_t) * LH2_LOCATIONS_COUNT);
            size_t length = sizeof(protocol_header_t) + sizeof(int16_t) + sizeof(db_lh2_raw_data_t) * LH2_LOCATIONS_COUNT;
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
        db_timer_delay_ms(100);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== private functions ================================

static void _update_control_loop(void) {
    if (_dotbot_vars.next_waypoint_idx >= _dotbot_vars.waypoints.length) {
        db_motors_set_speed(0, 0);
        return;
    }
    float dx               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].x / 1e6) - _dotbot_vars.last_location.x;
    float dy               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].y / 1e6) - _dotbot_vars.last_location.y;
    float distanceToTarget = sqrtf(powf(dx, 2) + powf(dy, 2));

    if (distanceToTarget < DB_WAYPOINT_DISTANCE_THRESHOLD) {
        // Target waypoint is reached
        _dotbot_vars.next_waypoint_idx++;
        return;
    }

    if (_dotbot_vars.direction == (int16_t)0xffff) {
        // Unknown direction, just move forward a bit
        db_motors_set_speed(DB_LINEAR_SPEED, DB_LINEAR_SPEED);
    } else {
        // compute angle to target waypoint
        int8_t sideFactor               = (dx > 0) ? -1 : 1;
        float  angleToTarget            = sideFactor * acosf(dy / distanceToTarget) * 180 / M_PI;
        _dotbot_vars.pid_angular.input  = (float)_dotbot_vars.direction;
        _dotbot_vars.pid_angular.target = angleToTarget;
        db_pid_update(&_dotbot_vars.pid_angular);
        float   angularSpeed = _dotbot_vars.pid_angular.output;
        int16_t left         = (int16_t)((DB_LINEAR_SPEED - angularSpeed) / 2);
        int16_t right        = (int16_t)((DB_LINEAR_SPEED + angularSpeed) / 2);
        db_motors_set_speed(left, right);
    }
}

static void _compute_direction(const dotbot_lh2_location_t *location, int16_t *direction) {
    float dx       = location->x - _dotbot_vars.last_location.x;
    float dy       = location->y - _dotbot_vars.last_location.y;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    if (distance < DB_DIRECTION_THRESHOLD) {
        // Skip computation if distance to last position is too small
        // Should we set an invalid direction here ?
        return;
    }

    int8_t sideFactor = (dx > 0) ? -1 : 1;
    *direction        = (int16_t)(acosf(dy / distance) * 180 / M_PI) * sideFactor;
    __NOP();  // For debugging
}

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
