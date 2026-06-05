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

#include <nrf.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "gpio.h"
#include "protocol.h"
#include "motors.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "control_loop.h"

//=========================== defines ==========================================

#define DB_RADIO_FREQ               (8U)      ///< Set the frequency to 2408 MHz
#define RADIO_APP                   (DotBot)  ///< DotBot Radio App
#define TIMER_DEV                   (0)
#define DB_POSITION_UPDATE_DELAY_MS (100U)   ///< 100ms delay between each LH2 position updates
#define DB_ADVERTIZEMENT_DELAY_MS   (500U)   ///< 500ms delay between each advertisement packet sending
#define DB_TIMEOUT_CHECK_DELAY_MS   (200U)   ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_TICKS   (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_BUFFER_MAX_BYTES         (255U)   ///< Max bytes in UART receive buffer
#define DB_LH2_OUTLIER_THRESHOLD    (500U)   ///< Max allowed displacement (mm) between two consecutive LH2 fixes

typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} position_2d_t;

typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    position_2d_t            last_position;                      ///< Last computed LH2 location received
    protocol_control_mode_t  control_mode;                       ///< Remote control mode
    protocol_lh2_waypoints_t waypoints;                          ///< List of waypoints
    volatile bool            update_control_loop;                ///< Whether the control loop need an update
    volatile bool            advertize;                          ///< Whether an advertize packet should be sent
    volatile bool            update_position;                    ///< Whether position must be updated
    uint64_t                 device_id;                          ///< Device ID of the DotBot
} dotbot_vars_t;

//============================= swarmit ========================================

typedef void (*ipc_isr_cb_t)(const uint8_t *, size_t);

// Swarmit NSC callbable functions
void swarmit_keep_alive(void);

void swarmit_send_raw_data(const uint8_t *packet, uint8_t length);
void swarmit_ipc_isr(ipc_isr_cb_t cb);

void swarmit_localization_get_position(position_2d_t *position);
void swarmit_get_battery_level(uint16_t *battery_level);
void swarmit_localization_handle_isr(void);

//=========================== variables ========================================

static dotbot_vars_t   _dotbot_vars  = { 0 };
static robot_control_t _control_vars = { 0 };
static void           *_control_ctx  = NULL;

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
static void _position_update(void);

//=========================== callbacks ========================================

static void _rx_data_callback(const uint8_t *pkt, size_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);
    uint8_t *cmd_ptr                     = (uint8_t *)pkt;
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
            if (count > DB_MAX_WAYPOINTS) {
                count = DB_MAX_WAYPOINTS;
            }
            memcpy(&_dotbot_vars.waypoints.points, cmd_ptr, count * sizeof(protocol_lh2_location_t));
            coordinate_t waypoints[DB_MAX_WAYPOINTS];
            for (uint8_t i = 0; i < count; i++) {
                waypoints[i].x = _dotbot_vars.waypoints.points[i].x;
                waypoints[i].y = _dotbot_vars.waypoints.points[i].y;
            }
            control_loop_set_waypoints(_control_ctx, waypoints, count, (uint32_t)threshold);
            _control_vars.encoder_left  = 0;
            _control_vars.encoder_right = 0;
            if (count > 0) {
                _dotbot_vars.control_mode = ControlAuto;
            } else {
                db_motors_set_speed(0, 0);
                _dotbot_vars.control_mode = ControlManual;
            }
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
    db_motors_init();
    db_gpio_init(&db_led1, DB_GPIO_OUT);
    _control_ctx = control_loop_alloc();

    // Set an invalid heading since the value is unknown on startup.
    // Control loop is stopped
    _control_vars.direction          = DB_DIRECTION_INVALID;
    _dotbot_vars.update_control_loop = false;
    _dotbot_vars.advertize           = false;
    _dotbot_vars.update_position     = false;

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(TIMER_DEV, 1, DB_POSITION_UPDATE_DELAY_MS, &_position_update);
    db_timer_set_periodic_ms(TIMER_DEV, 2, DB_ADVERTIZEMENT_DELAY_MS, &_advertise);

    while (1) {
        __WFE();

        if (_dotbot_vars.update_position) {
            _dotbot_vars.update_position = false;
            swarmit_keep_alive();
            swarmit_localization_get_position(&_dotbot_vars.last_position);

            if (_dotbot_vars.last_position.x > 100000 || _dotbot_vars.last_position.y > 100000) {
                // Invalid coordinates, do not update direction and upload position
                continue;
            }

            coordinate_t location = {
                .x = _dotbot_vars.last_position.x,
                .y = _dotbot_vars.last_position.y,
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
        }

        if (_dotbot_vars.advertize) {
            size_t length                       = 0;
            _dotbot_vars.radio_buffer[length++] = DB_PROTOCOL_DOTBOT_ADVERTISEMENT;
            // calibrated bitmask hard-coded 0xff: secure side doesn't expose per-LH calibration state via NSC yet.
            _dotbot_vars.radio_buffer[length++] = 0xff;
            memcpy(&_dotbot_vars.radio_buffer[length], &_control_vars.direction, sizeof(int16_t));
            length += sizeof(int16_t);
            protocol_lh2_location_t position = {
                .x = _control_vars.pos_x,
                .y = _control_vars.pos_y,
            };
            memcpy(&_dotbot_vars.radio_buffer[length], &position, sizeof(protocol_lh2_location_t));
            length += sizeof(protocol_lh2_location_t);
            uint16_t battery_level = 0;
            swarmit_get_battery_level(&battery_level);
            memcpy(&_dotbot_vars.radio_buffer[length], &battery_level, sizeof(uint16_t));
            length += sizeof(uint16_t);
            // pwm_left, pwm_right, mode, encoder_left, encoder_right,
            // waypoint_x, waypoint_y, waypoint_idx — zeroed placeholders
            // until plumbed in, kept to satisfy PyDotBot's full
            // DOTBOT_ADVERTISEMENT layout.
            memset(&_dotbot_vars.radio_buffer[length], 0, 20);
            length += 20;
            swarmit_send_raw_data(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.advertize = false;
        }
    }
}

//=========================== private functions ================================

static void _update_control_loop(void) {
    update_control(&_control_vars, _control_ctx);
    db_motors_set_speed(_control_vars.pwm_left, _control_vars.pwm_right);

    if (_control_vars.all_done) {
        _dotbot_vars.control_mode = ControlManual;
    }
}

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks(TIMER_DEV);
    if (_dotbot_vars.control_mode != ControlAuto && ticks > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        db_motors_set_speed(0, 0);
    }
}

static void _advertise(void) {
    db_gpio_toggle(&db_led1);
    _dotbot_vars.advertize = true;
}

static void _position_update(void) {
    _dotbot_vars.update_position = true;
}

void IPC_IRQHandler(void) {
    swarmit_ipc_isr(_rx_data_callback);
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
