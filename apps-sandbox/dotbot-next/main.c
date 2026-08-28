/**
 * @file
 * @defgroup project_dotbot_next    DotBot control application, rebuilt
 * @ingroup projects
 * @brief The sandboxed DotBot application, rebuilt in layers.
 *
 * Successor to apps-sandbox/dotbot, built up one layer at a time rather than
 * edited in place. At this stage it carries keepalive, position, encoders,
 * telemetry and direct motor commands, and no control layer above them.
 *
 * @copyright Inria, 2026
 */

#include <nrf.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "motors.h"
#include "protocol.h"
#include "qdec.h"
#include "rgbled_pwm.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV  (0)
#define QDEC_LEFT  (0)  ///< Left wheel QDEC peripheral index
#define QDEC_RIGHT (1)  ///< Right wheel QDEC peripheral index

/// One periodic tick drives everything; slower activities divide it down.
#define TICK_MS            (10U)
#define TICKS_PER_POSITION (10U)  ///< 100 ms, and the rate a new solve is published at
#define TICKS_PER_TIMEOUT  (20U)  ///< 200 ms
#define TICKS_PER_ADVERT   (50U)  ///< 500 ms

#define DB_BUFFER_MAX_BYTES (255U)
#define TIMEOUT_STOP_TICKS  (17000U)  ///< ~500 ms of RTC ticks without a command

/// NRF_RTC->COUNTER is 24 bits at 32768 Hz, so it wraps every 512 s and a plain
/// `now > then + delay` comparison is false for the whole pass after a wrap.
#define DB_RTC_COUNTER_MASK (0x00FFFFFFU)

/// Coordinates above this are the secure side reporting no usable solve.
#define POSITION_INVALID_MM (100000U)

/// Heading is not estimated here. The advertisement field carries the same
/// sentinel the control loop uses for "unknown", so hosts need no special case.
#define DIRECTION_INVALID (-1000)

#if defined(DB_BENCH_TELEMETRY)
/// Bench-only frame. Deliberately not added to protocol_data_type_t: it must not
/// exist in a shipped target. 13 is the one gap in that enum; see the README.
#define DB_PROTOCOL_BENCH_TELEMETRY (13)
#endif

typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} position_2d_t;

typedef struct {
    uint8_t       radio_buffer[DB_BUFFER_MAX_BYTES];
    uint32_t      ts_last_packet_received;  ///< RTC ticks at the last command received
    position_2d_t position;                 ///< Last solve accepted from the secure side
    uint32_t      fix_sequence;             ///< Sequence of the last solve seen; 0 before the first
    bool          has_position;             ///< False until the first in-bounds solve
    uint32_t      encoder_total_left;       ///< Counts since boot; wraps, and deltas are taken modulo 2^32
    uint32_t      encoder_total_right;      ///< Counts since boot; wraps, and deltas are taken modulo 2^32
    int8_t        pwm_left;                 ///< Last commanded duty, reported back for telemetry
    int8_t        pwm_right;                ///< Last commanded duty, reported back for telemetry
    uint32_t      max_tick_backlog;         ///< Worst number of ticks the main loop fell behind
} bench_vars_t;

/// One consumer's position in the encoder totals. Each consumer keeps its own,
/// so reading counts does not take them from anybody else.
typedef struct {
    uint32_t left;   ///< Left total at this consumer's last read
    uint32_t right;  ///< Right total at this consumer's last read
} encoder_cursor_t;

//============================= swarmit ========================================

typedef void (*ipc_isr_cb_t)(const uint8_t *, size_t);

// Swarmit NSC callable functions
void     swarmit_keep_alive(void);
void     swarmit_send_raw_data(const uint8_t *packet, uint8_t length);
void     swarmit_ipc_isr(ipc_isr_cb_t cb);
uint32_t swarmit_localization_get_fix(position_2d_t *position);
void     swarmit_get_battery_level(uint16_t *battery_level);
void     swarmit_localization_handle_isr(void);

//=========================== variables ========================================

static bench_vars_t _vars = { 0 };

/// The advertisement's own cursor into the encoder totals.
static encoder_cursor_t _advertisement_encoders = { 0 };

static volatile uint32_t _tick_count    = 0;  ///< Written by the tick callback only
static uint32_t          _tick_serviced = 0;  ///< Read and written by the main loop only

#ifdef DB_RGB_LED_PWM_RED_PORT
static const db_rgbled_pwm_conf_t _rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

#ifdef DB_QDEC_LEFT_A_PORT
static const qdec_conf_t _qdec_left_conf = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t _qdec_right_conf = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};
#endif

//=========================== prototypes =======================================

static void _tick(void);
static void _service_tick(uint32_t tick);
static void _encoders_init(void);
static void _encoders_accumulate(void);
static void _encoders_delta(encoder_cursor_t *cursor, int32_t *left, int32_t *right);
static void _position_poll(void);
static void _timeout_check(void);
static void _advertise(void);
static void _set_motors(int16_t left, int16_t right);

static inline uint32_t _ticks_since(uint32_t then) {
    return (db_timer_ticks(TIMER_DEV) - then) & DB_RTC_COUNTER_MASK;
}

//=========================== callbacks ========================================

static void _rx_data_callback(const uint8_t *pkt, size_t len) {
    (void)len;

    _vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);
    uint8_t *cmd_ptr              = (uint8_t *)pkt;

    switch ((uint8_t)*cmd_ptr++) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            _set_motors(left, right);
        } break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
#ifdef DB_RGB_LED_PWM_RED_PORT
            protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
#endif
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            _set_motors(0, 0);
            break;
        default:
            break;
    }
}

//=========================== main =============================================

int main(void) {
    db_board_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&_rgbled_pwm_conf);
#endif
    db_motors_init();
    _encoders_init();
    db_gpio_init(&db_led1, DB_GPIO_OUT);

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, TICK_MS, &_tick);

    while (1) {
        __WFE();

        uint32_t now    = _tick_count;
        uint32_t missed = now - _tick_serviced;
        if (missed == 0) {
            continue;
        }
        // Drop the backlog rather than working through it: a late tick is more
        // useful reported than replayed. The worst case is telemetered.
        if (missed - 1 > _vars.max_tick_backlog) {
            _vars.max_tick_backlog = missed - 1;
        }
        _tick_serviced = now;
        _service_tick(now);
    }
}

//=========================== private functions ================================

static void _tick(void) {
    _tick_count++;
}

static void _service_tick(uint32_t tick) {
    _encoders_accumulate();

    if (tick % TICKS_PER_POSITION == 0) {
        _position_poll();
    }
    if (tick % TICKS_PER_TIMEOUT == 0) {
        _timeout_check();
    }
    if (tick % TICKS_PER_ADVERT == 0) {
        _advertise();
    }
}

static void _encoders_init(void) {
#ifdef DB_QDEC_LEFT_A_PORT
    db_qdec_init(QDEC_LEFT, &_qdec_left_conf, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right_conf, NULL, NULL);
#endif
}

/// The hardware read is destructive, so the tick drains it into totals that are
/// never cleared. Consumers take deltas against their own cursor instead, which
/// is what lets telemetry and an estimator both see every count. On a board
/// without encoders the totals stay at zero.
static void _encoders_accumulate(void) {
#ifdef DB_QDEC_LEFT_A_PORT
    _vars.encoder_total_left += (uint32_t)db_qdec_read_and_clear(QDEC_LEFT);
    _vars.encoder_total_right += (uint32_t)db_qdec_read_and_clear(QDEC_RIGHT);
#endif
}

/// Counts since this cursor last read, leaving the totals for other consumers.
static void _encoders_delta(encoder_cursor_t *cursor, int32_t *left, int32_t *right) {
    uint32_t total_left  = _vars.encoder_total_left;
    uint32_t total_right = _vars.encoder_total_right;
    *left                = (int32_t)(total_left - cursor->left);
    *right               = (int32_t)(total_right - cursor->right);
    cursor->left         = total_left;
    cursor->right        = total_right;
}

/// swarmit_keep_alive() is what runs the solve and republishes shared data, so
/// this call sets the position rate and must immediately precede the read.
/// Takes the solve exactly as reported: no displacement gate, no heading
/// derivation, both belong above this layer.
static void _position_poll(void) {
    swarmit_keep_alive();

    position_2d_t solve    = { 0 };
    uint32_t      sequence = swarmit_localization_get_fix(&solve);

    // An unchanged sequence is the previous solve read a second time. Comparing
    // coordinates instead reads a stationary robot as having no new fix.
    if (sequence == _vars.fix_sequence) {
        return;
    }
    _vars.fix_sequence = sequence;

    if (solve.x > POSITION_INVALID_MM || solve.y > POSITION_INVALID_MM) {
        return;
    }
    _vars.position     = solve;
    _vars.has_position = true;
}

/// Unconditional: this app has no autonomous mode, so silence always means stop.
static void _timeout_check(void) {
    if (_ticks_since(_vars.ts_last_packet_received) > TIMEOUT_STOP_TICKS) {
        _set_motors(0, 0);
    }
}

static void _set_motors(int16_t left, int16_t right) {
    db_motors_set_speed(left, right);
    _vars.pwm_left  = (int8_t)left;
    _vars.pwm_right = (int8_t)right;
}

/// Byte-for-byte the layout apps-sandbox/dotbot emits, so the host parser is
/// unchanged. Fields this app does not own carry their unknown-value sentinels.
static void _advertise(void) {
    db_gpio_toggle(&db_led1);

    size_t   length = 0;
    uint8_t *buf    = _vars.radio_buffer;

    buf[length++] = DB_PROTOCOL_DOTBOT_ADVERTISEMENT;
    buf[length++] = 0xff;  // calibrated bitmask; the secure side exposes no per-LH state over NSC

    int16_t direction = DIRECTION_INVALID;
    memcpy(&buf[length], &direction, sizeof(int16_t));
    length += sizeof(int16_t);

    protocol_lh2_location_t position = {
        .x = _vars.has_position ? _vars.position.x : 0,
        .y = _vars.has_position ? _vars.position.y : 0,
    };
    memcpy(&buf[length], &position, sizeof(protocol_lh2_location_t));
    length += sizeof(protocol_lh2_location_t);

    uint16_t battery_level = 0;
    swarmit_get_battery_level(&battery_level);
    memcpy(&buf[length], &battery_level, sizeof(uint16_t));
    length += sizeof(uint16_t);

    buf[length++] = (uint8_t)_vars.pwm_left;
    buf[length++] = (uint8_t)_vars.pwm_right;
    buf[length++] = (uint8_t)ControlManual;

    int32_t encoder_left;
    int32_t encoder_right;
    _encoders_delta(&_advertisement_encoders, &encoder_left, &encoder_right);
    memcpy(&buf[length], &encoder_left, sizeof(int32_t));
    length += sizeof(int32_t);
    memcpy(&buf[length], &encoder_right, sizeof(int32_t));
    length += sizeof(int32_t);

    uint32_t waypoint = 0;
    memcpy(&buf[length], &waypoint, sizeof(uint32_t));
    length += sizeof(uint32_t);
    memcpy(&buf[length], &waypoint, sizeof(uint32_t));
    length += sizeof(uint32_t);
    buf[length++] = 0;  // waypoint index

    swarmit_send_raw_data(buf, (uint8_t)length);

#if defined(DB_BENCH_TELEMETRY)
    length        = 0;
    buf[length++] = DB_PROTOCOL_BENCH_TELEMETRY;

    uint32_t ticks = db_timer_ticks(TIMER_DEV);
    memcpy(&buf[length], &ticks, sizeof(uint32_t));
    length += sizeof(uint32_t);
    memcpy(&buf[length], &_tick_serviced, sizeof(uint32_t));
    length += sizeof(uint32_t);
    memcpy(&buf[length], &_vars.max_tick_backlog, sizeof(uint32_t));
    length += sizeof(uint32_t);
    _vars.max_tick_backlog = 0;

    memcpy(&buf[length], &position, sizeof(protocol_lh2_location_t));
    length += sizeof(protocol_lh2_location_t);
    memcpy(&buf[length], &_vars.fix_sequence, sizeof(uint32_t));
    length += sizeof(uint32_t);
    buf[length++] = (uint8_t)_vars.has_position;
    memcpy(&buf[length], &encoder_left, sizeof(int32_t));
    length += sizeof(int32_t);
    memcpy(&buf[length], &encoder_right, sizeof(int32_t));
    length += sizeof(int32_t);

    swarmit_send_raw_data(buf, (uint8_t)length);
#endif
}

void IPC_IRQHandler(void) {
    swarmit_ipc_isr(_rx_data_callback);
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
