/**
 * @file
 * @defgroup project_dotbot_next    DotBot control application, rebuilt
 * @ingroup projects
 * @brief Sandboxed DotBot app: keepalive, position, encoders, telemetry,
 * direct motor commands and the per-wheel speed loop.
 *
 * @copyright Inria, 2026
 */

#include <nrf.h>
#include <stdbool.h>
#include <stddef.h>
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
#include "wheel_control.h"

//=========================== defines ==========================================

#define TIMER_DEV  (0)
#define QDEC_LEFT  (0)  ///< Left wheel QDEC peripheral index
#define QDEC_RIGHT (1)  ///< Right wheel QDEC peripheral index

/// One periodic tick drives everything; slower activities divide it down.
#define TICK_MS            (10U)
#define TICKS_PER_POSITION (10U)  ///< 100 ms, and the rate a new solve is published at
#define TICKS_PER_TIMEOUT  (20U)  ///< 200 ms

/// Adverts take this share of the node's transmit slots, one per minimum TX
/// interval, and the rest is left for the net core's STATUS frame
#define ADVERT_TX_SHARE_PERCENT (50U)
#define ADVERT_PERIOD_MIN_MS    (100U)   ///< Floor, however short the interval
#define ADVERT_PERIOD_MAX_MS    (1000U)  ///< Ceiling, however long the interval
#define ADVERT_PERIOD_DEF_MS    (500U)   ///< While not joined, when the interval reads 0

_Static_assert(TICK_MS == DB_WHEEL_CONTROL_TICK_MS, "the wheel loop's dt assumes this tick");

/// Largest wheel speed a command may set, in mm/s
#define WHEEL_SPEED_MAX_MM_S (800)

/// Room for the largest command this app accepts, header byte included
#define RX_MAILBOX_BYTES (16U)

#define DB_BUFFER_MAX_BYTES (255U)
#define TIMEOUT_STOP_TICKS  (17000U)  ///< ~500 ms of RTC ticks without a command

/// NRF_RTC->COUNTER is 24 bits at 32768 Hz, so it wraps every 512 s and a plain
/// `now > then + delay` comparison is false for the whole pass after a wrap.
#define DB_RTC_COUNTER_MASK (0x00FFFFFFU)

/// Coordinates above this are the secure side reporting no usable solve.
#define POSITION_INVALID_MM (100000U)

/// Heading is not estimated here; this is the advertisement's "unknown" value.
#define DIRECTION_INVALID (-1000)

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
/// Duty the bench records carry for a braked motor, outside [-100, 100]
#define PWM_BRAKED (INT8_MIN)
#endif

#if defined(DB_BENCH_TELEMETRY)
/// Bench-only frame type; keep it out of protocol_data_type_t, where 13 is unused.
#define DB_PROTOCOL_BENCH_TELEMETRY (13)
#define TELEMETRY_STEPS             (24U)  ///< Steps one frame carries, the newest kept
#define TELEMETRY_FIXES             (4U)   ///< New solves one frame carries, the newest kept
#endif

/// Who writes the motors. Exactly one writer per mode.
typedef enum {
    DRIVE_IDLE,      ///< Nothing commanded; the wheel loop at zero brakes a turning wheel, else coasts
    DRIVE_RAW,       ///< MOVE_RAW writes duty directly and the wheel loop is off
    DRIVE_VELOCITY,  ///< The wheel loop is the only writer
} drive_mode_t;

typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} position_2d_t;
_Static_assert(sizeof(position_2d_t) == 8, "must match the bootloader's layout");
_Static_assert(offsetof(position_2d_t, y) == 4, "must match the bootloader's layout");

typedef struct {
    uint8_t       radio_buffer[DB_BUFFER_MAX_BYTES];
    uint32_t      ts_last_packet_received;  ///< RTC ticks at the last command received
    position_2d_t position;                 ///< Last solve accepted from the secure side
    uint32_t      fix_sequence;             ///< Sequence of the last solve seen; 0 before the first
    bool          has_position;             ///< False until the first in-bounds solve
    uint32_t      encoder_total_left;       ///< Counts since boot; wraps, and deltas are taken modulo 2^32
    uint32_t      encoder_total_right;      ///< Counts since boot; wraps, and deltas are taken modulo 2^32
    uint32_t      double_total_left;        ///< Double transitions since boot, already credited in the totals
    uint32_t      double_total_right;       ///< Double transitions since boot, already credited in the totals
    drive_mode_t  drive_mode;               ///< Which writer owns the motors
    int8_t        pwm_left;                 ///< Last commanded duty, reported back for telemetry
    int8_t        pwm_right;                ///< Last commanded duty, reported back for telemetry
    bool          brake_left;               ///< Left motor shorted, its duty ignored
    bool          brake_right;              ///< Right motor shorted, its duty ignored
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
uint32_t swarmit_get_min_tx_interval_us(void);

//=========================== variables ========================================

static bench_vars_t _vars = { 0 };

/// The advertisement's own cursor into the encoder totals.
static encoder_cursor_t _advertisement_encoders = { 0 };

static volatile uint32_t _tick_count    = 0;  ///< Written by the tick callback only
static uint32_t          _tick_serviced = 0;  ///< Read and written by the main loop only
static uint32_t          _tick_position = 0;  ///< Tick of the last position poll, main loop only
static uint32_t          _tick_timeout  = 0;  ///< Tick of the last timeout check, main loop only
static uint32_t          _tick_advert   = 0;  ///< Tick of the last advertisement, main loop only
static uint32_t          _tick_wheel    = 0;  ///< Tick of the last wheel step, main loop only

/// The wheel loop's own cursor into the encoder totals
static encoder_cursor_t _wheel_encoders = { 0 };

/// Feedforward from an untethered open-loop duty sweep of one v3 on the office
/// carpet (breakaway 37-45, rolling at 32 + 0.092..0.101 duty per mm/s for
/// either wheel and direction); ki from closed-loop holds, kp from step
/// responses. Full duty, and no slew limit short of it.
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
};
static db_wheel_control_t _wheel_left;
static db_wheel_control_t _wheel_right;

/// Commands arrive in the IPC interrupt and are applied on the next tick, so
/// the main loop is the only writer of the drive state and the motors.
static uint8_t       _rx_buffer[RX_MAILBOX_BYTES];
static size_t        _rx_length  = 0;
static volatile bool _rx_pending = false;

#if defined(DB_BENCH_TRACE)
/// One tick while anything drives the motors, read back over the debugger
typedef struct __attribute__((packed)) {
    uint32_t tick;            ///< Serviced tick
    int16_t  setpoint_left;   ///< mm/s
    int16_t  setpoint_right;  ///< mm/s
    int16_t  counts_left;     ///< Credited counts over this step
    int16_t  counts_right;    ///< Credited counts over this step
    int8_t   pwm_left;        ///< Duty written, PWM_BRAKED while braked
    int8_t   pwm_right;       ///< Duty written, PWM_BRAKED while braked
    uint8_t  elapsed;         ///< Ticks this step covered
    uint8_t  mode;            ///< drive_mode_t at this step
} wheel_trace_t;

#define TRACE_LENGTH     (1000U)  ///< 10 s of steps
#define TRACE_TAIL_TICKS (100U)   ///< Keep recording this long after the loop stops

__attribute__((used)) static wheel_trace_t _trace[TRACE_LENGTH];
__attribute__((used)) static uint32_t      _trace_count = 0;
static uint32_t                            _trace_tail  = 0;

/// Every new solve the secure side publishes, in a ring, whatever the drive
/// mode: fix rate and jitter come from the sequence against the tick
typedef struct __attribute__((packed)) {
    uint32_t tick;      ///< Serviced tick the solve was read on
    uint32_t sequence;  ///< Fix sequence
    uint32_t x;         ///< mm, as reported, before the bounds check
    uint32_t y;         ///< mm, as reported, before the bounds check
} fix_trace_t;

#define FIX_TRACE_LENGTH (3000U)  ///< 5 minutes at 10 Hz

__attribute__((used)) static fix_trace_t _fix_trace[FIX_TRACE_LENGTH];
__attribute__((used)) static uint32_t    _fix_trace_count = 0;  ///< Total written; the ring index is this modulo the length
#endif

#if defined(DB_BENCH_TELEMETRY)
/// One wheel step, as the telemetry frame carries it
typedef struct __attribute__((packed)) {
    int8_t counts_left;     ///< Credited counts over this step, saturated
    int8_t counts_right;    ///< Credited counts over this step, saturated
    int8_t pwm_left;        ///< Duty written, PWM_BRAKED while braked
    int8_t pwm_right;       ///< Duty written, PWM_BRAKED while braked
    int8_t setpoint_left;   ///< In units of 10 mm/s
    int8_t setpoint_right;  ///< In units of 10 mm/s
} telemetry_step_t;

/// One new solve, as the telemetry frame carries it
typedef struct __attribute__((packed)) {
    uint16_t tick;      ///< Low 16 bits of the serviced tick the solve was read on
    uint16_t sequence;  ///< Low 16 bits of the fix sequence
    uint16_t x;         ///< mm, before the bounds check, saturated
    uint16_t y;         ///< mm, before the bounds check, saturated
} telemetry_fix_t;

static telemetry_step_t _telemetry_steps[TELEMETRY_STEPS];
static uint32_t         _telemetry_step_count = 0;  ///< Steps recorded; the ring index is this modulo the length
static uint32_t         _telemetry_step_sent  = 0;  ///< Step count at the last frame
static uint32_t         _telemetry_step_tick  = 0;  ///< Serviced tick of the newest step
static telemetry_fix_t  _telemetry_fixes[TELEMETRY_FIXES];
static uint32_t         _telemetry_fix_count = 0;  ///< Solves recorded; the ring index is this modulo the length
static uint32_t         _telemetry_fix_sent  = 0;  ///< Solve count at the last frame
#endif

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

static void     _tick(void);
static void     _service_tick(uint32_t tick);
static void     _encoders_init(void);
static void     _encoders_accumulate(void);
static void     _encoders_delta(encoder_cursor_t *cursor, int32_t *left, int32_t *right);
static void     _position_poll(void);
static void     _timeout_check(void);
static void     _advertise(void);
static uint32_t _advert_period_ticks(void);
static void     _set_motors(int16_t left, int16_t right, bool brake_left, bool brake_right);
static void     _rx_process(void);
static void     _drive_stop(void);
static void     _wheel_service(uint32_t tick);

/// Elapsed rather than a multiple, since the main loop drops its backlog and
/// can step over any given tick.
static inline bool _due(uint32_t *last, uint32_t tick, uint32_t period) {
    if (tick - *last < period) {
        return false;
    }
    *last = tick;
    return true;
}

static inline uint32_t _ticks_since(uint32_t then) {
    return (db_timer_ticks(TIMER_DEV) - then) & DB_RTC_COUNTER_MASK;
}

//=========================== callbacks ========================================

static void _rx_data_callback(const uint8_t *pkt, size_t len) {
    _vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);

    // One command per tick is plenty; a second one before the tick is dropped
    if (_rx_pending || len == 0 || len > sizeof(_rx_buffer)) {
        return;
    }
    memcpy(_rx_buffer, pkt, len);
    _rx_length = len;
    __DMB();  // the buffer is complete before the flag says so
    _rx_pending = true;
}

//=========================== main =============================================

int main(void) {
    db_board_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&_rgbled_pwm_conf);
#endif
    db_motors_init();
    _encoders_init();
    db_wheel_control_init(&_wheel_left, &_wheel_conf);
    db_wheel_control_init(&_wheel_right, &_wheel_conf);
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
        // The backlog is dropped, not replayed; the worst case is telemetered
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
    _rx_process();
    _encoders_accumulate();
    _wheel_service(tick);

    if (_due(&_tick_position, tick, TICKS_PER_POSITION)) {
        _position_poll();
    }
    if (_due(&_tick_timeout, tick, TICKS_PER_TIMEOUT)) {
        _timeout_check();
    }
    if (_due(&_tick_advert, tick, _advert_period_ticks())) {
        _advertise();
    }
}

static void _rx_process(void) {
    if (!_rx_pending) {
        return;
    }
    __DMB();  // read the buffer only after seeing the flag
    uint8_t packet[RX_MAILBOX_BYTES];
    size_t  length = _rx_length;
    memcpy(packet, _rx_buffer, length);
    _rx_pending = false;

    const uint8_t *payload = &packet[1];
    switch (packet[0]) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            if (length < 1 + sizeof(protocol_move_raw_command_t)) {
                break;
            }
            protocol_move_raw_command_t command;
            memcpy(&command, payload, sizeof(command));
#if defined(DB_BENCH_TRACE)
            if (_vars.drive_mode == DRIVE_IDLE) {
                _trace_count = 0;
            }
#endif
            _vars.drive_mode = DRIVE_RAW;
            db_wheel_control_reset(&_wheel_left);
            db_wheel_control_reset(&_wheel_right);
            _set_motors((int16_t)(100 * ((float)command.left_y / INT8_MAX)), (int16_t)(100 * ((float)command.right_y / INT8_MAX)), false, false);
        } break;
        case DB_PROTOCOL_CMD_WHEEL_VELOCITY:
        {
            if (length < 1 + sizeof(protocol_wheel_velocity_command_t)) {
                break;
            }
            protocol_wheel_velocity_command_t command;
            memcpy(&command, payload, sizeof(command));
            if (_vars.drive_mode != DRIVE_VELOCITY) {
                db_wheel_control_reset(&_wheel_left);
                db_wheel_control_reset(&_wheel_right);
#if defined(DB_BENCH_TRACE)
                if (_vars.drive_mode == DRIVE_IDLE) {
                    _trace_count = 0;
                }
#endif
                _vars.drive_mode = DRIVE_VELOCITY;
            }
            int16_t left  = command.left_mm_s;
            int16_t right = command.right_mm_s;
            left          = (left > WHEEL_SPEED_MAX_MM_S) ? WHEEL_SPEED_MAX_MM_S : ((left < -WHEEL_SPEED_MAX_MM_S) ? -WHEEL_SPEED_MAX_MM_S : left);
            right         = (right > WHEEL_SPEED_MAX_MM_S) ? WHEEL_SPEED_MAX_MM_S : ((right < -WHEEL_SPEED_MAX_MM_S) ? -WHEEL_SPEED_MAX_MM_S : right);
            db_wheel_control_set_setpoint(&_wheel_left, left);
            db_wheel_control_set_setpoint(&_wheel_right, right);
        } break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
#ifdef DB_RGB_LED_PWM_RED_PORT
            if (length < 1 + sizeof(protocol_rgbled_command_t)) {
                break;
            }
            protocol_rgbled_command_t command;
            memcpy(&command, payload, sizeof(command));
            db_rgbled_pwm_set_color(command.r, command.g, command.b);
#endif
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            _drive_stop();
            break;
        default:
            break;
    }
}

/// Brakes both motors; from the next tick the wheel loop releases each one once
/// its wheel stands
static void _drive_stop(void) {
    _vars.drive_mode = DRIVE_IDLE;
    db_wheel_control_reset(&_wheel_left);
    db_wheel_control_reset(&_wheel_right);
    _set_motors(0, 0, true, true);
}

#if defined(DB_BENCH_TELEMETRY)
static inline int8_t _saturate_i8(int32_t value) {
    return (value > INT8_MAX) ? INT8_MAX : ((value < INT8_MIN) ? INT8_MIN : (int8_t)value);
}
#endif

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
static inline int8_t _pwm_recorded(int8_t pwm, bool brake) {
    return brake ? PWM_BRAKED : pwm;
}
#endif

/// Runs on every tick so the cursor never lags, and writes the motors only
/// while the loop owns them: driving by velocity, and after a stop, where its
/// zero setpoints brake the wheels until they stand
static void _wheel_service(uint32_t tick) {
    uint32_t elapsed = tick - _tick_wheel;
    _tick_wheel      = tick;
    int32_t left;
    int32_t right;
    _encoders_delta(&_wheel_encoders, &left, &right);

    if (_vars.drive_mode != DRIVE_RAW) {
        int8_t pwm_left  = db_wheel_control_step(&_wheel_left, left, elapsed);
        int8_t pwm_right = db_wheel_control_step(&_wheel_right, right, elapsed);
        _set_motors(pwm_left, pwm_right, _wheel_left.brake, _wheel_right.brake);
    }

#if defined(DB_BENCH_TELEMETRY)
    _telemetry_steps[_telemetry_step_count % TELEMETRY_STEPS] = (telemetry_step_t){
        .counts_left    = _saturate_i8(left),
        .counts_right   = _saturate_i8(right),
        .pwm_left       = _pwm_recorded(_vars.pwm_left, _vars.brake_left),
        .pwm_right      = _pwm_recorded(_vars.pwm_right, _vars.brake_right),
        .setpoint_left  = _saturate_i8((int32_t)_wheel_left.setpoint / 10),
        .setpoint_right = _saturate_i8((int32_t)_wheel_right.setpoint / 10),
    };
    _telemetry_step_count++;
    _telemetry_step_tick = tick;
#endif

#if defined(DB_BENCH_TRACE)
    if (_vars.drive_mode != DRIVE_IDLE) {
        _trace_tail = TRACE_TAIL_TICKS;
    } else if (_trace_tail > 0) {
        _trace_tail--;
    } else {
        return;
    }
    if (_trace_count < TRACE_LENGTH) {
        _trace[_trace_count++] = (wheel_trace_t){
            .tick           = tick,
            .setpoint_left  = (int16_t)_wheel_left.setpoint,
            .setpoint_right = (int16_t)_wheel_right.setpoint,
            .counts_left    = (int16_t)left,
            .counts_right   = (int16_t)right,
            .pwm_left       = _pwm_recorded(_vars.pwm_left, _vars.brake_left),
            .pwm_right      = _pwm_recorded(_vars.pwm_right, _vars.brake_right),
            .elapsed        = (uint8_t)elapsed,
            .mode           = (uint8_t)_vars.drive_mode,
        };
    }
#endif
}

/// Derived from the node's minimum TX interval at every advert, so a gateway on
/// another schedule changes the rate within one period
static uint32_t _advert_period_ticks(void) {
    uint32_t min_tx_interval_us = swarmit_get_min_tx_interval_us();
    uint32_t period_ms          = ADVERT_PERIOD_DEF_MS;
    if (min_tx_interval_us > 0) {
        period_ms = (min_tx_interval_us / 1000U) * 100U / ADVERT_TX_SHARE_PERCENT;
        if (period_ms < ADVERT_PERIOD_MIN_MS) {
            period_ms = ADVERT_PERIOD_MIN_MS;
        } else if (period_ms > ADVERT_PERIOD_MAX_MS) {
            period_ms = ADVERT_PERIOD_MAX_MS;
        }
    }
#if defined(DB_BENCH_TELEMETRY)
    // Each advert is two frames
    period_ms *= 2;
#endif
    return period_ms / TICK_MS;
}

static void _encoders_init(void) {
#ifdef DB_QDEC_LEFT_A_PORT
    db_qdec_init(QDEC_LEFT, &_qdec_left_conf, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right_conf, NULL, NULL);
#endif
}

/// The hardware read is destructive, so the tick drains it into totals that are
/// never cleared; consumers take deltas against their own cursor.
static void _encoders_accumulate(void) {
#ifdef DB_QDEC_LEFT_A_PORT
    uint32_t dbl_left;
    uint32_t dbl_right;
    int32_t  acc_left  = db_qdec_read_and_clear_dbl(QDEC_LEFT, &dbl_left);
    int32_t  acc_right = db_qdec_read_and_clear_dbl(QDEC_RIGHT, &dbl_right);
    _vars.encoder_total_left += (uint32_t)db_wheel_control_counts(acc_left, dbl_left);
    _vars.encoder_total_right += (uint32_t)db_wheel_control_counts(acc_right, dbl_right);
    _vars.double_total_left += dbl_left;
    _vars.double_total_right += dbl_right;
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

/// swarmit_keep_alive() runs the solve; call it immediately before reading the fix.
static void _position_poll(void) {
    swarmit_keep_alive();

    position_2d_t solve    = { 0 };
    uint32_t      sequence = swarmit_localization_get_fix(&solve);

    // An unchanged sequence is the previous solve read a second time
    if (sequence == _vars.fix_sequence) {
        return;
    }
    _vars.fix_sequence = sequence;

#if defined(DB_BENCH_TRACE)
    _fix_trace[_fix_trace_count % FIX_TRACE_LENGTH] = (fix_trace_t){
        .tick     = _tick_serviced,
        .sequence = sequence,
        .x        = solve.x,
        .y        = solve.y,
    };
    _fix_trace_count++;
#endif
#if defined(DB_BENCH_TELEMETRY)
    _telemetry_fixes[_telemetry_fix_count % TELEMETRY_FIXES] = (telemetry_fix_t){
        .tick     = (uint16_t)_tick_serviced,
        .sequence = (uint16_t)sequence,
        .x        = (solve.x > UINT16_MAX) ? UINT16_MAX : (uint16_t)solve.x,
        .y        = (solve.y > UINT16_MAX) ? UINT16_MAX : (uint16_t)solve.y,
    };
    _telemetry_fix_count++;
#endif

    if (solve.x > POSITION_INVALID_MM || solve.y > POSITION_INVALID_MM) {
        return;
    }
    _vars.position     = solve;
    _vars.has_position = true;
}

/// Raw and velocity driving both stop when the host goes silent.
static void _timeout_check(void) {
    if (_vars.drive_mode != DRIVE_IDLE && _ticks_since(_vars.ts_last_packet_received) > TIMEOUT_STOP_TICKS) {
        _drive_stop();
    }
}

static void _set_motors(int16_t left, int16_t right, bool brake_left, bool brake_right) {
    db_motors_set_pwm_brake(left, right, brake_left, brake_right);
    _vars.pwm_left    = brake_left ? 0 : (int8_t)left;
    _vars.pwm_right   = brake_right ? 0 : (int8_t)right;
    _vars.brake_left  = brake_left;
    _vars.brake_right = brake_right;
}

static void _put(uint8_t *buf, size_t *length, const void *value, size_t size) {
    memcpy(&buf[*length], value, size);
    *length += size;
}

#if defined(DB_BENCH_TELEMETRY)
static inline uint8_t _saturate_u8(uint32_t value) {
    return (value > UINT8_MAX) ? UINT8_MAX : (uint8_t)value;
}

/// Every step and every new solve since the previous frame, newest kept when
/// there are more than a frame holds; resets the worst tick backlog it reports.
/// Layout: type, newest step tick (u32), steps carried, steps dropped, backlog,
/// drive mode, encoder totals (i32 x 2), solves carried, solves dropped, then
/// the steps oldest first, then the solves oldest first.
static void _send_bench_telemetry(void) {
    size_t   length = 0;
    uint8_t *buf    = _vars.radio_buffer;

    uint32_t steps     = _telemetry_step_count - _telemetry_step_sent;
    uint32_t steps_out = (steps > TELEMETRY_STEPS) ? TELEMETRY_STEPS : steps;
    uint32_t fixes     = _telemetry_fix_count - _telemetry_fix_sent;
    uint32_t fixes_out = (fixes > TELEMETRY_FIXES) ? TELEMETRY_FIXES : fixes;

    buf[length++] = DB_PROTOCOL_BENCH_TELEMETRY;
    _put(buf, &length, &_telemetry_step_tick, sizeof(_telemetry_step_tick));
    buf[length++]          = (uint8_t)steps_out;
    buf[length++]          = _saturate_u8(steps - steps_out);
    buf[length++]          = _saturate_u8(_vars.max_tick_backlog);
    _vars.max_tick_backlog = 0;
    buf[length++]          = (uint8_t)_vars.drive_mode;
    _put(buf, &length, &_vars.encoder_total_left, sizeof(_vars.encoder_total_left));
    _put(buf, &length, &_vars.encoder_total_right, sizeof(_vars.encoder_total_right));
    buf[length++] = (uint8_t)fixes_out;
    buf[length++] = _saturate_u8(fixes - fixes_out);

    for (uint32_t i = _telemetry_step_count - steps_out; i != _telemetry_step_count; i++) {
        _put(buf, &length, &_telemetry_steps[i % TELEMETRY_STEPS], sizeof(telemetry_step_t));
    }
    for (uint32_t i = _telemetry_fix_count - fixes_out; i != _telemetry_fix_count; i++) {
        _put(buf, &length, &_telemetry_fixes[i % TELEMETRY_FIXES], sizeof(telemetry_fix_t));
    }
    _telemetry_step_sent = _telemetry_step_count;
    _telemetry_fix_sent  = _telemetry_fix_count;

    swarmit_send_raw_data(buf, (uint8_t)length);
}
#endif

/// Layout of DB_PROTOCOL_DOTBOT_ADVERTISEMENT; keep in step with apps-sandbox/dotbot.
/// Fields this app does not own carry their unknown-value sentinels.
static void _advertise(void) {
    db_gpio_toggle(&db_led1);

    size_t   length = 0;
    uint8_t *buf    = _vars.radio_buffer;

    buf[length++] = DB_PROTOCOL_DOTBOT_ADVERTISEMENT;
    buf[length++] = 0xff;  // calibrated bitmask, unknown

    int16_t direction = DIRECTION_INVALID;
    _put(buf, &length, &direction, sizeof(direction));

    protocol_lh2_location_t position = {
        .x = _vars.has_position ? _vars.position.x : 0,
        .y = _vars.has_position ? _vars.position.y : 0,
    };
    _put(buf, &length, &position, sizeof(position));

    uint16_t battery_level = 0;
    swarmit_get_battery_level(&battery_level);
    _put(buf, &length, &battery_level, sizeof(battery_level));

    buf[length++] = (uint8_t)_vars.pwm_left;
    buf[length++] = (uint8_t)_vars.pwm_right;
    buf[length++] = (uint8_t)ControlManual;

    int32_t encoder_left;
    int32_t encoder_right;
    _encoders_delta(&_advertisement_encoders, &encoder_left, &encoder_right);
    _put(buf, &length, &encoder_left, sizeof(encoder_left));
    _put(buf, &length, &encoder_right, sizeof(encoder_right));

    uint32_t waypoint = 0;
    _put(buf, &length, &waypoint, sizeof(waypoint));
    _put(buf, &length, &waypoint, sizeof(waypoint));
    buf[length++] = 0;  // waypoint index

    swarmit_send_raw_data(buf, (uint8_t)length);

#if defined(DB_BENCH_TELEMETRY)
    _send_bench_telemetry();
#endif
}

void IPC_IRQHandler(void) {
    swarmit_ipc_isr(_rx_data_callback);
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
