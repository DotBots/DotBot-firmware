/**
 * @file
 * @defgroup project_sandbox_dotbot    DotBot control application
 * @ingroup projects
 * @brief Sandboxed DotBot app: the hardware around drv/dotbot_control, which
 * runs the per-wheel speed loop, the pose estimator and steering along a batch
 * of waypoints. The app owns the tick, the command mailbox, the encoders, the
 * LH2 fix and keepalive, the motors, the advertisement and the RGB LED.
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
#include "dotbot_control.h"
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

#define DB_BUFFER_MAX_BYTES (255U)

_Static_assert(DB_CONTROL_ADVERTISEMENT_BYTES <= DB_BUFFER_MAX_BYTES, "the advertisement fits the radio buffer");

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
/// Duty the bench records carry for a braked motor, outside [-100, 100]
#define PWM_BRAKED (INT8_MIN)
/// Duty the bench records carry for a wheel the loop has stalled, outside [-100, 100]
#define PWM_STALLED (INT8_MIN + 1)
#endif

#if defined(DB_BENCH_TELEMETRY)
/// Bench-only frame type; keep it out of protocol_data_type_t, where 13 is unused.
#define DB_PROTOCOL_BENCH_TELEMETRY (13)
#define TELEMETRY_STEPS             (24U)  ///< Steps one frame carries, the newest kept
#define TELEMETRY_FIXES             (4U)   ///< New solves one frame carries, the newest kept
#endif

typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} position_2d_t;
_Static_assert(sizeof(position_2d_t) == 8, "must match the bootloader's layout");
_Static_assert(offsetof(position_2d_t, y) == 4, "must match the bootloader's layout");

typedef struct {
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];
    uint32_t double_total_left;   ///< Double transitions since boot, already credited in the counts
    uint32_t double_total_right;  ///< Double transitions since boot, already credited in the counts
    uint32_t max_tick_backlog;    ///< Worst number of ticks the main loop fell behind
} bench_vars_t;

//============================= swarmit ========================================

typedef void (*ipc_isr_cb_t)(const uint8_t *, size_t);

// Swarmit NSC callable functions
void     swarmit_keep_alive(void);
void     swarmit_send_raw_data(const uint8_t *packet, uint8_t length);
void     swarmit_ipc_isr(ipc_isr_cb_t cb);
uint32_t swarmit_localization_get_fix(position_2d_t *position);
uint8_t  swarmit_localization_get_lines(db_lh2_floor_line_t *lines, uint8_t max);
void     swarmit_get_battery_level(uint16_t *battery_level);
void     swarmit_localization_handle_isr(void);
uint32_t swarmit_get_min_tx_interval_us(void);

//=========================== variables ========================================

static bench_vars_t _vars = { 0 };

static volatile uint32_t _tick_count    = 0;  ///< Written by the tick callback only
static uint32_t          _tick_serviced = 0;  ///< Read and written by the main loop only

/// Read over the debugger for the estimator's counters and covariance, and the
/// steering's state and failure reason
__attribute__((used)) static db_control_t _control;

#if defined(DB_BENCH_TELEMETRY)
/// The default steering with the spin recovery, selected by a bench command
static db_steering_conf_t _steering_conf_spin;
#endif

/// Commands arrive in the IPC interrupt and are applied on the next tick, so
/// the main loop is the only caller of the control core.
static uint8_t       _rx_buffer[DB_CONTROL_RX_MAX_BYTES];
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
    int8_t   pwm_left;        ///< Duty written, PWM_BRAKED while braked, PWM_STALLED while stalled
    int8_t   pwm_right;       ///< Duty written, PWM_BRAKED while braked, PWM_STALLED while stalled
    uint8_t  elapsed;         ///< Ticks this step covered
    uint8_t  mode;            ///< db_control_drive_mode_t at the end of this step
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
    int8_t pwm_left;        ///< Duty written, PWM_BRAKED while braked, PWM_STALLED while stalled
    int8_t pwm_right;       ///< Duty written, PWM_BRAKED while braked, PWM_STALLED while stalled
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

static void _tick(void);
static void _service_tick(uint32_t tick, uint32_t elapsed);
static void _encoders_init(void);
static void _encoders_read(int32_t *left, int32_t *right);
static void _position_read(db_control_input_t *in);
static void _rx_process(void);
static void _set_led(const uint8_t *command, size_t length);
static void _advert_period_update(void);
static void _advertise(void);
#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
static void _bench_record_step(uint32_t tick, uint32_t elapsed, const db_control_input_t *in);
#endif

//=========================== callbacks ========================================

/// Runs in the IPC interrupt: the LED is set here, and the newest other
/// command replaces one the main loop has not applied yet
static void _rx_data_callback(const uint8_t *pkt, size_t len) {
    if (len == 0 || len > sizeof(_rx_buffer)) {
        return;
    }
    if (pkt[0] == DB_PROTOCOL_CMD_RGB_LED) {
        _set_led(pkt, len);
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
    db_control_init(&_control, &db_control_default_conf);
#if defined(DB_BENCH_TELEMETRY)
    _steering_conf_spin         = db_control_default_conf.steering;
    _steering_conf_spin.recover = DB_STEERING_RECOVER_SPIN;
#endif
    db_gpio_init(&db_led1, DB_GPIO_OUT);

    _advert_period_update();
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_CONTROL_TICK_MS, &_tick);

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
        _service_tick(now, missed);
    }
}

//=========================== private functions ================================

static void _tick(void) {
    _tick_count++;
}

static void _service_tick(uint32_t tick, uint32_t elapsed) {
    _rx_process();

    db_control_input_t in = {
        .fix_sequence  = _control.fix_sequence,
        .elapsed_ticks = elapsed,
    };
    _encoders_read(&in.counts_left, &in.counts_right);
    if (db_control_fix_due(&_control, elapsed)) {
        _position_read(&in);
    }

    db_control_output_t out;
    db_control_tick(&_control, &in, &out);
    if (out.write) {
        db_motors_set_pwm_brake(out.pwm_left, out.pwm_right, out.brake_left, out.brake_right);
    }
#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
    _bench_record_step(tick, elapsed, &in);
#else
    (void)tick;
#endif
    if (out.advertise) {
        _advertise();
    }
}

#if defined(DB_BENCH_TELEMETRY)
/// Bench only: a threshold of 0xFFFF drops the estimator's pose, as a kidnap
/// does, to exercise the steering's heading recovery; 0xFFFE does the same with
/// the spin recovery, until the next waypoint. True when the batch was one of these.
static bool _bench_waypoints(const uint8_t *payload, size_t length) {
    db_steering_path_t path;
    uint8_t            batch_id;
    if (!db_steering_path_from_wire(payload, length, &path, &batch_id)) {
        return false;
    }
    if (path.threshold_mm >= (float)(UINT16_MAX - 1)) {
        _control.steering.conf = (path.threshold_mm == (float)UINT16_MAX) ? &_control.conf->steering : &_steering_conf_spin;
        db_pose_estimator_init(&_control.estimator, &_control.conf->estimator);
        return true;
    }
    _control.steering.conf = &_control.conf->steering;
    return false;
}
#endif

static void _rx_process(void) {
    if (!_rx_pending) {
        return;
    }
    // Masked so the IPC interrupt cannot replace the buffer mid-copy
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();  // read the buffer only after seeing the flag
    uint8_t packet[DB_CONTROL_RX_MAX_BYTES];
    size_t  length = _rx_length;
    memcpy(packet, _rx_buffer, length);
    __DMB();  // the copy is complete before the flag frees the buffer
    _rx_pending = false;
    __set_PRIMASK(primask);

#if defined(DB_BENCH_TELEMETRY)
    if (packet[0] == DB_PROTOCOL_LH2_WAYPOINTS && _bench_waypoints(&packet[1], length - 1)) {
        return;
    }
#endif
#if defined(DB_BENCH_TRACE)
    db_control_drive_mode_t mode = _control.drive_mode;
#endif
    db_control_rx(&_control, packet, length);
#if defined(DB_BENCH_TRACE)
    if (mode == DB_CONTROL_DRIVE_IDLE && _control.drive_mode != DB_CONTROL_DRIVE_IDLE) {
        _trace_count = 0;
    }
#endif
}

#if defined(DB_BENCH_TELEMETRY)
static inline int8_t _saturate_i8(int32_t value) {
    return (value > INT8_MAX) ? INT8_MAX : ((value < INT8_MIN) ? INT8_MIN : (int8_t)value);
}
#endif

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
static inline int8_t _pwm_recorded(int8_t pwm, bool brake, bool stalled) {
    return brake ? PWM_BRAKED : (stalled ? PWM_STALLED : pwm);
}

/// One control tick, after the core ran it
static void _bench_record_step(uint32_t tick, uint32_t elapsed, const db_control_input_t *in) {
    int8_t pwm_left  = _pwm_recorded(_control.pwm_left, _control.brake_left, _control.wheel_left.stalled);
    int8_t pwm_right = _pwm_recorded(_control.pwm_right, _control.brake_right, _control.wheel_right.stalled);
#if defined(DB_BENCH_TELEMETRY)
    _telemetry_steps[_telemetry_step_count % TELEMETRY_STEPS] = (telemetry_step_t){
        .counts_left    = _saturate_i8(in->counts_left),
        .counts_right   = _saturate_i8(in->counts_right),
        .pwm_left       = pwm_left,
        .pwm_right      = pwm_right,
        .setpoint_left  = _saturate_i8((int32_t)_control.wheel_left.setpoint / 10),
        .setpoint_right = _saturate_i8((int32_t)_control.wheel_right.setpoint / 10),
    };
    _telemetry_step_count++;
    _telemetry_step_tick = tick;
#endif

#if defined(DB_BENCH_TRACE)
    if (_control.drive_mode != DB_CONTROL_DRIVE_IDLE) {
        _trace_tail = TRACE_TAIL_TICKS;
    } else if (_trace_tail > 0) {
        _trace_tail--;
    } else {
        return;
    }
    if (_trace_count < TRACE_LENGTH) {
        _trace[_trace_count++] = (wheel_trace_t){
            .tick           = tick,
            .setpoint_left  = (int16_t)_control.wheel_left.setpoint,
            .setpoint_right = (int16_t)_control.wheel_right.setpoint,
            .counts_left    = (int16_t)in->counts_left,
            .counts_right   = (int16_t)in->counts_right,
            .pwm_left       = pwm_left,
            .pwm_right      = pwm_right,
            .elapsed        = (uint8_t)elapsed,
            .mode           = (uint8_t)_control.drive_mode,
        };
    }
#else
    (void)elapsed;
#endif
}
#endif

/// From the node's minimum TX interval, so a gateway on another schedule changes
/// the rate within one period
static void _advert_period_update(void) {
    db_control_set_min_tx_interval(&_control, swarmit_get_min_tx_interval_us());
#if defined(DB_BENCH_TELEMETRY)
    // Each advert is two frames
    _control.advert_period_ticks *= 2;
#endif
}

static void _encoders_init(void) {
#ifdef DB_QDEC_LEFT_A_PORT
    db_qdec_init(QDEC_LEFT, &_qdec_left_conf, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right_conf, NULL, NULL);
#endif
}

/// Counts since the previous read, doubles credited; the hardware read is destructive
static void _encoders_read(int32_t *left, int32_t *right) {
    *left  = 0;
    *right = 0;
#ifdef DB_QDEC_LEFT_A_PORT
    uint32_t dbl_left;
    uint32_t dbl_right;
    int32_t  acc_left  = db_qdec_read_and_clear_dbl(QDEC_LEFT, &dbl_left);
    int32_t  acc_right = db_qdec_read_and_clear_dbl(QDEC_RIGHT, &dbl_right);
    *left              = db_wheel_control_counts(acc_left, dbl_left);
    *right             = db_wheel_control_counts(acc_right, dbl_right);
    _vars.double_total_left += dbl_left;
    _vars.double_total_right += dbl_right;
#endif
}

/// swarmit_keep_alive() runs the solve; call it immediately before reading
/// the fix and the floor lines of the same sweeps, which the next tick fuses.
static void _position_read(db_control_input_t *in) {
    swarmit_keep_alive();

    position_2d_t solve = { 0 };
    in->fix_sequence    = swarmit_localization_get_fix(&solve);
    in->fix_x           = solve.x;
    in->fix_y           = solve.y;

    db_lh2_floor_line_t lines[DB_CONTROL_LINES_MAX];
    db_control_lines(&_control, lines, swarmit_localization_get_lines(lines, DB_CONTROL_LINES_MAX));

#if defined(DB_BENCH_TRACE) || defined(DB_BENCH_TELEMETRY)
    // An unchanged sequence is the previous solve read a second time
    if (in->fix_sequence == _control.fix_sequence) {
        return;
    }
#endif
#if defined(DB_BENCH_TRACE)
    _fix_trace[_fix_trace_count % FIX_TRACE_LENGTH] = (fix_trace_t){
        .tick     = _tick_serviced,
        .sequence = in->fix_sequence,
        .x        = solve.x,
        .y        = solve.y,
    };
    _fix_trace_count++;
#endif
#if defined(DB_BENCH_TELEMETRY)
    _telemetry_fixes[_telemetry_fix_count % TELEMETRY_FIXES] = (telemetry_fix_t){
        .tick     = (uint16_t)_tick_serviced,
        .sequence = (uint16_t)in->fix_sequence,
        .x        = (solve.x > UINT16_MAX) ? UINT16_MAX : (uint16_t)solve.x,
        .y        = (solve.y > UINT16_MAX) ? UINT16_MAX : (uint16_t)solve.y,
    };
    _telemetry_fix_count++;
#endif
}

#if defined(DB_BENCH_TELEMETRY)
static void _put(uint8_t *buf, size_t *length, const void *value, size_t size) {
    memcpy(&buf[*length], value, size);
    *length += size;
}

static inline uint8_t _saturate_u8(uint32_t value) {
    return (value > UINT8_MAX) ? UINT8_MAX : (uint8_t)value;
}

/// Every step and every new solve since the previous frame, newest kept when
/// there are more than a frame holds; resets the worst tick backlog it reports.
/// Layout: type, newest step tick (u32), steps carried, steps dropped, backlog,
/// drive mode in the low nibble with the steering state in the high one,
/// encoder totals (i32 x 2), solves carried, solves dropped, then the steps
/// oldest first, then the solves oldest first, then the estimator: status, the
/// last gated fix's squared distance x 10 (u16, saturated), and its kidnap and
/// re-anchor counts (u8 each, wrapping), then the steering's point index and
/// corrections made (u8 each).
static void _send_bench_telemetry(void) {
    size_t                     length    = 0;
    uint8_t                   *buf       = _vars.radio_buffer;
    const db_pose_estimator_t *estimator = &_control.estimator;
    const db_steering_t       *steering  = &_control.steering;

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
    buf[length++]          = (uint8_t)(_control.drive_mode | (steering->state << 4));
    _put(buf, &length, &_control.encoder_left, sizeof(_control.encoder_left));
    _put(buf, &length, &_control.encoder_right, sizeof(_control.encoder_right));
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

    buf[length++]  = (uint8_t)estimator->status;
    float    d2    = estimator->last_d2 * 10.0f;
    uint16_t d2x10 = (d2 >= (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)d2;
    _put(buf, &length, &d2x10, sizeof(d2x10));
    buf[length++] = (uint8_t)estimator->kidnaps;
    buf[length++] = (uint8_t)estimator->reanchors;
    buf[length++] = steering->index;
    buf[length++] = (uint8_t)steering->nudges;

    swarmit_send_raw_data(buf, (uint8_t)length);
}
#endif

/// command is the type byte followed by its payload
static void _set_led(const uint8_t *command, size_t length) {
#ifdef DB_RGB_LED_PWM_RED_PORT
    protocol_rgbled_command_t color;
    if (length < 1 + sizeof(color)) {
        return;
    }
    memcpy(&color, &command[1], sizeof(color));
    db_rgbled_pwm_set_color(color.r, color.g, color.b);
#else
    (void)command;
    (void)length;
#endif
}

static void _advertise(void) {
    _advert_period_update();
    db_gpio_toggle(&db_led1);

    uint16_t battery_level = 0;
    swarmit_get_battery_level(&battery_level);
    size_t length = db_control_advertisement(&_control, battery_level, _vars.radio_buffer);
    swarmit_send_raw_data(_vars.radio_buffer, (uint8_t)length);

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
