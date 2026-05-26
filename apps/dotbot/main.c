/**
 * @file
 * @defgroup project_dotbot    DotBot application
 * @ingroup projects
 * @brief This is the radio-controlled DotBot app
 *
 * The remote control can be either a keyboard, a joystick or buttons on the gateway
 * itself
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <nrf.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "motors.h"
#include "qdec.h"
#include "radio.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "log_flash.h"
#include "tdma_client.h"
#include "battery.h"
#include "control_loop.h"

//=========================== defines ==========================================

#define DB_RADIO_FREQ             (8U)      ///< Set the frequency to 2408 MHz
#define RADIO_APP                 (DotBot)  ///< DotBot Radio App
#define TIMER_DEV                 (0)
#define QDEC_LEFT                 (0)      ///< Left wheel QDEC peripheral index
#define QDEC_RIGHT                (1)      ///< Right wheel QDEC peripheral index
#define DB_LH2_UPDATE_DELAY_MS    (50U)    ///< 50ms delay between each LH2 data refresh
#define DB_ADVERTIZEMENT_DELAY_MS (500U)   ///< 500ms delay between each advertizement packet sending
#define DB_TIMEOUT_CHECK_DELAY_MS (200U)   ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_BUFFER_MAX_BYTES       (255U)   ///< Max bytes in UART receive buffer
#define DB_LH2_OUTLIER_THRESHOLD  (500U)   ///< Max allowed displacement (mm) between two consecutive LH2 fixes

typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t                 lh2;                                ///< LH2 device descriptor
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    protocol_control_mode_t  control_mode;                       ///< Remote control mode
    protocol_lh2_waypoints_t waypoints;                          ///< List of waypoints
    bool                     update_control_loop;                ///< Whether the control loop need an update
    bool                     advertize;                          ///< Whether an advertize packet should be sent
    bool                     update_lh2;                         ///< Whether LH2 data must be processed
    uint64_t                 device_id;                          ///< Device ID of the DotBot
    double                   coordinates[2];                     ///< x, y coordinates of the robot
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t   _dotbot_vars  = { 0 };
static robot_control_t _control_vars = { 0 };
static void           *_control_ctx  = NULL;

static const qdec_conf_t _qdec_left_conf = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t _qdec_right_conf = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

#ifdef DB_RGB_LED_PWM_RED_PORT  // Only available on DotBot v2
static const db_rgbled_pwm_conf_t rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

//=========================== prototypes =======================================

static void _timeout_check(void);
static void _advertise(void);
static void _update_control_loop(void);
static void _update_lh2(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);
    uint8_t           *ptk_ptr           = pkt;
    protocol_header_t *header            = (protocol_header_t *)ptk_ptr;
    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _dotbot_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and update the motors' speeds
    switch ((uint8_t)*cmd_ptr++) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            _control_vars.pwm_left               = left;
            _control_vars.pwm_right              = right;
            db_motors_set_speed(left, right);
        } break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
            protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            db_motors_set_speed(0, 0);
            break;
        case DB_PROTOCOL_LH2_WAYPOINTS:
        {
            _dotbot_vars.control_mode = ControlManual;
            uint16_t threshold        = 0;
            memcpy(&threshold, cmd_ptr, sizeof(uint16_t));
            cmd_ptr += sizeof(uint16_t);
            uint8_t count = (uint8_t)*cmd_ptr++;
            memcpy(&_dotbot_vars.waypoints.points, cmd_ptr, count * sizeof(protocol_lh2_location_t));
            coordinate_t waypoints[DB_MAX_WAYPOINTS];
            for (uint8_t i = 0; i < count; i++) {
                waypoints[i].x = _dotbot_vars.waypoints.points[i].x;
                waypoints[i].y = _dotbot_vars.waypoints.points[i].y;
            }
            control_loop_set_waypoints(_control_ctx, waypoints, count, (uint32_t)threshold);
            _control_vars.encoder_left  = 0;
            _control_vars.encoder_right = 0;
            db_qdec_read_and_clear(QDEC_LEFT);
            db_qdec_read_and_clear(QDEC_RIGHT);
            if (count > 0) {
                _dotbot_vars.control_mode = ControlAuto;
            } else {
                db_motors_set_speed(0, 0);
                _dotbot_vars.control_mode = ControlManual;
            }
        } break;
        case DB_PROTOCOL_LH2_CALIBRATION:
        {
            puts("Received calibration data");
            protocol_lh2_homography_t *homography_from_packet = (protocol_lh2_homography_t *)cmd_ptr;
            db_lh2_store_homography(&_dotbot_vars.lh2, homography_from_packet->basestation_index, homography_from_packet->homography_matrix);
        } break;
        default:
            break;
    }
}

//=========================== main =============================================

int main(void) {
    db_board_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&rgbled_pwm_conf);
#endif
    db_battery_level_init();
    db_motors_init();
    db_qdec_init(QDEC_LEFT, &_qdec_left_conf, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right_conf, NULL, NULL);
    _control_ctx = control_loop_alloc();
    db_tdma_client_init(&radio_callback, DB_RADIO_BLE_1MBit, DB_RADIO_FREQ);

    // Set an invalid heading since the value is unknown on startup.
    // Control loop is stopped
    _control_vars.direction          = DB_DIRECTION_INVALID;
    _dotbot_vars.update_control_loop = false;
    _dotbot_vars.advertize           = false;
    _dotbot_vars.update_lh2          = false;

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(TIMER_DEV, 1, 5 * DB_LH2_UPDATE_DELAY_MS, &_update_lh2);
    db_timer_set_periodic_ms(TIMER_DEV, 2, DB_ADVERTIZEMENT_DELAY_MS, &_advertise);
    db_lh2_init(&_dotbot_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    while (1) {
        __WFE();

        // Process available lighthouse data
        db_lh2_process_location(&_dotbot_vars.lh2);

        if (_dotbot_vars.update_lh2) {
            _dotbot_vars.update_lh2 = false;
            db_lh2_stop();
            for (uint8_t lh_index = 0; lh_index < LH2_BASESTATION_COUNT; lh_index++) {
                if (_dotbot_vars.lh2.lh2_calibration_complete[lh_index] && _dotbot_vars.lh2.data_ready[0][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE && _dotbot_vars.lh2.data_ready[1][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE) {
                    db_lh2_calculate_position(_dotbot_vars.lh2.locations[0][lh_index].lfsr_counts, _dotbot_vars.lh2.locations[1][lh_index].lfsr_counts, lh_index, _dotbot_vars.coordinates);
                    _dotbot_vars.lh2.data_ready[0][lh_index] = DB_LH2_NO_NEW_DATA;
                    _dotbot_vars.lh2.data_ready[1][lh_index] = DB_LH2_NO_NEW_DATA;
                    break;
                }
            }
            db_lh2_start();

            if (_dotbot_vars.coordinates[0] < 0 || _dotbot_vars.coordinates[1] < 0 || _dotbot_vars.coordinates[0] > 100000 || _dotbot_vars.coordinates[1] > 100000) {
                // Invalid coordinates, do not update direction and upload position
                continue;
            }

            coordinate_t location = {
                .x = (uint32_t)(_dotbot_vars.coordinates[0]),
                .y = (uint32_t)(_dotbot_vars.coordinates[1]),
            };
            coordinate_t last_location = { .x = _control_vars.pos_x, .y = _control_vars.pos_y };
            float        dlx           = (float)location.x - (float)last_location.x;
            float        dly           = (float)location.y - (float)last_location.y;
            if (_control_vars.pos_x != 0 && _control_vars.pos_y != 0 &&
                sqrtf(dlx * dlx + dly * dly) > DB_LH2_OUTLIER_THRESHOLD) {
                continue;
            }
            int16_t angle = _control_vars.direction;
            if (compute_angle(&last_location, &location, &angle)) {
                _control_vars.direction = angle;
                _control_vars.pos_x     = location.x;
                _control_vars.pos_y     = location.y;
            }
            _dotbot_vars.update_control_loop = (_dotbot_vars.control_mode == ControlAuto);
        }

        if (_dotbot_vars.update_control_loop) {
            _update_control_loop();
            _dotbot_vars.update_control_loop = false;
#if defined(ENABLE_DOTBOT_LOG_DATA)
            _dotbot_vars.advertize = true;
#endif
        }

        if (_dotbot_vars.advertize) {
            uint8_t calibration_complete = 0;
            for (uint8_t i = 0; i < LH2_BASESTATION_COUNT; i++) {
                if (_dotbot_vars.lh2.lh2_calibration_complete[i]) {
                    calibration_complete |= (1 << i);
                }
            }
            size_t                  length    = db_protocol_dotbot_advertizement_to_buffer(_dotbot_vars.radio_buffer, DB_GATEWAY_ADDRESS, calibration_complete);
            int16_t                 direction = 0xFFFF;
            protocol_lh2_location_t position  = {
                 .x = 0xffffffff,
                 .y = 0xffffffff,
            };
            if (calibration_complete) {
                direction  = _control_vars.direction;
                position.x = _control_vars.pos_x;
                position.y = _control_vars.pos_y;
            }
            memcpy(&_dotbot_vars.radio_buffer[length], &direction, sizeof(int16_t));
            length += sizeof(int16_t);
            memcpy(&_dotbot_vars.radio_buffer[length], &position, sizeof(protocol_lh2_location_t));
            length += sizeof(protocol_lh2_location_t);
            uint16_t battery_level_mv = db_battery_level_read();
            memcpy(&_dotbot_vars.radio_buffer[length], &battery_level_mv, sizeof(uint16_t));
            length += sizeof(uint16_t);
            memcpy(&_dotbot_vars.radio_buffer[length++], &_control_vars.pwm_left, sizeof(int8_t));
            memcpy(&_dotbot_vars.radio_buffer[length++], &_control_vars.pwm_right, sizeof(int8_t));
            memcpy(&_dotbot_vars.radio_buffer[length++], &_dotbot_vars.control_mode, sizeof(uint8_t));
            memcpy(&_dotbot_vars.radio_buffer[length], &_control_vars.encoder_left, sizeof(int32_t));
            length += sizeof(int32_t);
            memcpy(&_dotbot_vars.radio_buffer[length], &_control_vars.encoder_right, sizeof(int32_t));
            length += sizeof(int32_t);
            memcpy(&_dotbot_vars.radio_buffer[length], &_control_vars.waypoint_x, sizeof(uint32_t));
            length += sizeof(uint32_t);
            memcpy(&_dotbot_vars.radio_buffer[length], &_control_vars.waypoint_y, sizeof(uint32_t));
            length += sizeof(uint32_t);
            memcpy(&_dotbot_vars.radio_buffer[length++], &_control_vars.waypoint_idx, sizeof(uint8_t));
            db_tdma_client_tx(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.advertize = false;
        }
    }
}

//=========================== private functions ================================

static void _update_control_loop(void) {
    _control_vars.encoder_left  = db_qdec_read_and_clear(QDEC_LEFT);
    _control_vars.encoder_right = db_qdec_read_and_clear(QDEC_RIGHT);
    update_control(&_control_vars, _control_ctx);
    db_motors_set_speed(_control_vars.pwm_left, _control_vars.pwm_right);

    if (_control_vars.all_done) {
        _dotbot_vars.control_mode   = ControlManual;
        _control_vars.encoder_left  = 0;
        _control_vars.encoder_right = 0;
    }
}

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks(TIMER_DEV);
    if (_dotbot_vars.control_mode != ControlAuto && ticks > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        db_motors_set_speed(0, 0);
    }
}

static void _advertise(void) {
    _dotbot_vars.advertize = true;
}

static void _update_lh2(void) {
    _dotbot_vars.update_lh2 = true;
}
