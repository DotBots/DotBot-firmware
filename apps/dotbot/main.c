/**
 * @file
 * @defgroup project_dotbot    DotBot application
 * @ingroup projects
 * @brief Radio-controlled DotBot: wheel speeds, motor duty and the RGB LED
 *
 * Commands come over the radio from a gateway running dotbot_gateway:
 * - WHEEL_VELOCITY sets a speed per wheel in mm/s, which a speed loop holds
 *   using the wheel encoders (drv/wheel_control);
 * - MOVE_RAW sets the motor duty directly, as a joystick or keyboard does;
 * - RGB_LED sets the LED colour;
 * - CONTROL_MODE stops the motors.
 *
 * The motors stop when no driving command has arrived for DEADMAN_TICKS. The
 * robot advertises its battery, motor duty and encoder counts every
 * ADVERTISEMENT_TICKS.
 *
 * @copyright Inria, 2022-2026
 */

#include <nrf.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "qdec.h"
#include "radio.h"
#include "timer.h"
// Include DRV headers
#include "battery.h"
#include "frame.h"
#include "motors.h"
#include "protocol.h"
#include "rgbled_pwm.h"
#include "wheel_control.h"

//=========================== defines ==========================================

#define TIMER_DEV           (0)
#define QDEC_LEFT           (0)                         ///< Left wheel QDEC peripheral index
#define QDEC_RIGHT          (1)                         ///< Right wheel QDEC peripheral index
#define TICK_MS             (DB_WHEEL_CONTROL_TICK_MS)  ///< 10 ms, the period of the speed loop
#define ADVERTISEMENT_TICKS (50U)                       ///< 500 ms between advertisements
#define DEADMAN_TICKS       (52U)                       ///< ~520 ms without a driving command stops the motors
#define SPEED_MAX_MM_S      (700)                       ///< Largest wheel speed a command may set
#define DIRECTION_NONE      (-1000)                     ///< Heading in the advertisement: this app has none
#define POSITION_NONE       (0xFFFFFFFF)                ///< Position in the advertisement: this app has none
#define CALIBRATION_UNKNOWN (0xFF)                      ///< LH2 calibration bitmask in the advertisement: not applicable
#define BUFFER_MAX_BYTES    (255U)

/// Who writes the motors
typedef enum {
    DRIVE_SPEED,  ///< The speed loop, toward the wheel setpoints; a stop is a zero setpoint
    DRIVE_RAW,    ///< MOVE_RAW duty, the speed loop off
} drive_mode_t;

/// What follows the calibration byte of DB_PROTOCOL_DOTBOT_ADVERTISEMENT
typedef struct __attribute__((packed)) {
    int16_t                 direction;       ///< Heading, deg
    protocol_lh2_location_t position;        ///< mm
    uint16_t                battery;         ///< mV
    int8_t                  pwm_left;        ///< Last duty written
    int8_t                  pwm_right;       ///< Last duty written
    uint8_t                 mode;            ///< protocol_control_mode_t
    int32_t                 encoder_left;    ///< Counts since the previous advertisement
    int32_t                 encoder_right;   ///< Counts since the previous advertisement
    protocol_lh2_location_t waypoint;        ///< Point being driven to, mm
    uint8_t                 waypoint_index;  ///< Index of that point
} advertisement_t;
_Static_assert(sizeof(advertisement_t) == 32, "advertisement_t is a wire format");

typedef struct {
    volatile uint32_t  tick;                         ///< Ticks since boot, written by the timer callback only
    uint32_t           tick_serviced;                ///< Last tick the main loop ran
    uint32_t           tick_command;                 ///< Tick of the last driving command
    uint32_t           tick_advertisement;           ///< Tick of the last advertisement
    uint8_t            rx_buffer[BUFFER_MAX_BYTES];  ///< Command received, type byte first
    size_t             rx_length;                    ///< Bytes in rx_buffer
    volatile bool      rx_pending;                   ///< A command waits in rx_buffer
    uint8_t            tx_buffer[BUFFER_MAX_BYTES];  ///< Advertisement being sent
    uint64_t           device_id;                    ///< This robot's address
    drive_mode_t       drive_mode;                   ///< Who writes the motors
    int8_t             pwm_left;                     ///< Last duty written, 0 while braked
    int8_t             pwm_right;                    ///< Last duty written, 0 while braked
    int32_t            encoder_left;                 ///< Counts since the last advertisement
    int32_t            encoder_right;                ///< Counts since the last advertisement
    db_wheel_control_t wheel_left;                   ///< Left wheel speed loop
    db_wheel_control_t wheel_right;                  ///< Right wheel speed loop
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars = { 0 };

/// DotBot v3 speed loop gains, as in drv/dotbot_control (the sandbox app); keep them in step
static const db_wheel_control_conf_t _wheel_conf = {
    .kp                = 0.52f,
    .ki                = 5.2f,
    .u_breakaway       = 44.0f,
    .kick_ramp         = 0.5f,
    .u_run             = 32.0f,
    .k_run             = 0.097f,
    .i_zone            = 38.0f,
    .pwm_max           = 100.0f,
    .pwm_slew_per_tick = 100.0f,
    .stall_pwm         = 80.0f,
    .stall_ms          = 500U,
};

static const qdec_conf_t _qdec_left_conf = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t _qdec_right_conf = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

#ifdef DB_RGB_LED_PWM_RED_PORT  // Not every board has the RGB LED
static const db_rgbled_pwm_conf_t _rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

//=========================== prototypes =======================================

static void _tick(void);
static void _service_tick(uint32_t elapsed);
static void _rx_process(void);
static void _apply_command(const uint8_t *command, size_t length);
static void _set_speed(int16_t left_mm_s, int16_t right_mm_s);
static void _set_raw(int8_t left, int8_t right);
static void _write_motors(int8_t left, int8_t right, bool brake_left, bool brake_right);
static void _advertise(void);

//=========================== callbacks ========================================

/// Runs in the radio interrupt: keep the command for the main loop, which is
/// the only place the motors and the speed loop are touched
static void _radio_callback(uint8_t *packet, uint8_t length) {
    if (length < sizeof(db_frame_header_t) + 1) {
        return;
    }
    const db_frame_header_t *header = (const db_frame_header_t *)packet;
    if (header->dst != DB_FRAME_DST_BROADCAST && header->dst != _dotbot_vars.device_id) {
        return;
    }
    if (header->version != DB_FRAME_VERSION ||
        header->type != DB_FRAME_TYPE_DATA ||
        header->next_proto != DB_FRAME_NEXT_PROTO) {
        return;
    }
    // A newer command replaces one the main loop has not applied yet
    _dotbot_vars.rx_length = length - sizeof(db_frame_header_t);
    memcpy(_dotbot_vars.rx_buffer, packet + sizeof(db_frame_header_t), _dotbot_vars.rx_length);
    _dotbot_vars.rx_pending = true;
}

static void _tick(void) {
    _dotbot_vars.tick++;
}

//=========================== main =============================================

int main(void) {
    db_board_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&_rgbled_pwm_conf);
#endif
    db_battery_level_init();
    db_motors_init();
    db_qdec_init(QDEC_LEFT, &_qdec_left_conf, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right_conf, NULL, NULL);
    db_wheel_control_init(&_dotbot_vars.wheel_left, &_wheel_conf);
    db_wheel_control_init(&_dotbot_vars.wheel_right, &_wheel_conf);
    _dotbot_vars.drive_mode = DRIVE_SPEED;
    _dotbot_vars.device_id  = db_device_id();

    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_network_address(DB_FRAME_ACCESS_ADDR);
    db_radio_set_frequency(DB_FRAME_DEFAULT_FREQ);
    db_radio_rx();

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, TICK_MS, &_tick);

    while (1) {
        __WFE();

        uint32_t now     = _dotbot_vars.tick;
        uint32_t elapsed = now - _dotbot_vars.tick_serviced;
        if (elapsed == 0) {
            continue;
        }
        _dotbot_vars.tick_serviced = now;
        _service_tick(elapsed);
    }
}

//=========================== private functions ================================

/// One step of everything periodic; elapsed is more than 1 if ticks were missed
static void _service_tick(uint32_t elapsed) {
    _rx_process();

    // Encoder counts since the previous step; the read clears the counter
    uint32_t dbl_left;
    uint32_t dbl_right;
    int32_t  left  = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_LEFT, &dbl_left), dbl_left);
    int32_t  right = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_RIGHT, &dbl_right), dbl_right);
    _dotbot_vars.encoder_left += left;
    _dotbot_vars.encoder_right += right;

    uint32_t tick = _dotbot_vars.tick_serviced;
    if (tick - _dotbot_vars.tick_command > DEADMAN_TICKS) {
        _set_speed(0, 0);
    }

    if (_dotbot_vars.drive_mode == DRIVE_SPEED) {
        int8_t pwm_left  = db_wheel_control_step(&_dotbot_vars.wheel_left, left, elapsed);
        int8_t pwm_right = db_wheel_control_step(&_dotbot_vars.wheel_right, right, elapsed);
        _write_motors(pwm_left, pwm_right, _dotbot_vars.wheel_left.brake, _dotbot_vars.wheel_right.brake);
    }

    if (tick - _dotbot_vars.tick_advertisement >= ADVERTISEMENT_TICKS) {
        _dotbot_vars.tick_advertisement = tick;
        _advertise();
    }
}

static void _rx_process(void) {
    if (!_dotbot_vars.rx_pending) {
        return;
    }
    // Masked so the radio interrupt cannot replace the command mid-copy
    uint8_t command[BUFFER_MAX_BYTES];
    __disable_irq();
    size_t length = _dotbot_vars.rx_length;
    memcpy(command, _dotbot_vars.rx_buffer, length);
    _dotbot_vars.rx_pending = false;
    __enable_irq();

    _apply_command(command, length);
}

/// command is the type byte followed by its payload
static void _apply_command(const uint8_t *command, size_t length) {
    const uint8_t *payload = &command[1];
    bool           driving = true;  // a driving command restarts the deadman
    switch (command[0]) {
        case DB_PROTOCOL_CMD_WHEEL_VELOCITY:
        {
            protocol_wheel_velocity_command_t speed;
            if (length < 1 + sizeof(speed)) {
                return;
            }
            memcpy(&speed, payload, sizeof(speed));
            _set_speed(speed.left_mm_s, speed.right_mm_s);
        } break;
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t raw;
            if (length < 1 + sizeof(raw)) {
                return;
            }
            memcpy(&raw, payload, sizeof(raw));
            // Joystick axes, -127 to 127, to duty, -100 to 100
            _set_raw((int8_t)(100 * raw.left_y / INT8_MAX), (int8_t)(100 * raw.right_y / INT8_MAX));
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            _set_speed(0, 0);
            break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
            driving = false;
#ifdef DB_RGB_LED_PWM_RED_PORT
            protocol_rgbled_command_t color;
            if (length < 1 + sizeof(color)) {
                return;
            }
            memcpy(&color, payload, sizeof(color));
            db_rgbled_pwm_set_color(color.r, color.g, color.b);
#endif
        } break;
        default:
            return;
    }
    if (driving) {
        _dotbot_vars.tick_command = _dotbot_vars.tick_serviced;
    }
}

static int16_t _clamp_speed(int16_t mm_s) {
    if (mm_s > SPEED_MAX_MM_S) {
        return SPEED_MAX_MM_S;
    }
    if (mm_s < -SPEED_MAX_MM_S) {
        return -SPEED_MAX_MM_S;
    }
    return mm_s;
}

/// Hand the motors to the speed loop, toward these wheel speeds
static void _set_speed(int16_t left_mm_s, int16_t right_mm_s) {
    if (_dotbot_vars.drive_mode != DRIVE_SPEED) {
        // Start from zero rather than from what the loop had before MOVE_RAW
        db_wheel_control_reset(&_dotbot_vars.wheel_left);
        db_wheel_control_reset(&_dotbot_vars.wheel_right);
        _dotbot_vars.drive_mode = DRIVE_SPEED;
    }
    db_wheel_control_set_setpoint(&_dotbot_vars.wheel_left, _clamp_speed(left_mm_s));
    db_wheel_control_set_setpoint(&_dotbot_vars.wheel_right, _clamp_speed(right_mm_s));
}

/// Take the motors from the speed loop and write this duty
static void _set_raw(int8_t left, int8_t right) {
    _dotbot_vars.drive_mode = DRIVE_RAW;
    _write_motors(left, right, false, false);
}

static void _write_motors(int8_t left, int8_t right, bool brake_left, bool brake_right) {
    db_motors_set_pwm_brake(left, right, brake_left, brake_right);
    _dotbot_vars.pwm_left  = brake_left ? 0 : left;
    _dotbot_vars.pwm_right = brake_right ? 0 : right;
}

/// The standard DotBot advertisement; the fields this app has no value for
/// (calibration, position, heading, waypoint) carry their unknown-value sentinels
static void _advertise(void) {
    uint8_t *buffer = _dotbot_vars.tx_buffer;
    size_t   length = db_protocol_dotbot_advertizement_to_buffer(buffer, DB_GATEWAY_ADDRESS, CALIBRATION_UNKNOWN);

    advertisement_t advertisement = {
        .direction      = DIRECTION_NONE,
        .position       = { .x = POSITION_NONE, .y = POSITION_NONE },
        .battery        = db_battery_level_read(),
        .pwm_left       = _dotbot_vars.pwm_left,
        .pwm_right      = _dotbot_vars.pwm_right,
        .mode           = ControlManual,
        .encoder_left   = _dotbot_vars.encoder_left,
        .encoder_right  = _dotbot_vars.encoder_right,
        .waypoint       = { .x = 0, .y = 0 },
        .waypoint_index = 0,
    };
    memcpy(&buffer[length], &advertisement, sizeof(advertisement));
    length += sizeof(advertisement);
    _dotbot_vars.encoder_left  = 0;
    _dotbot_vars.encoder_right = 0;

    db_radio_disable();
    db_radio_tx(buffer, (uint8_t)length);
    db_radio_rx();
}
