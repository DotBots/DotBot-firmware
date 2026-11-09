/**
 * @file
 * @ingroup project_sandbox_dotbot
 * @brief Bench-only recorders of the sandbox DotBot app; see bench.h.
 *
 * @copyright Inria, 2026
 */

#include "bench.h"

#if defined(DB_BENCH)

#include <math.h>
#include <string.h>
#include "protocol.h"

//=========================== defines ==========================================

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

#if defined(DB_BENCH_FUSION)
/// Bench-only frame type, kept out of protocol_data_type_t as DB_PROTOCOL_BENCH_TELEMETRY is;
/// far above the enum, whose 14 is the calibration and whose 16 the host reads as raw data
#define DB_PROTOCOL_BENCH_FUSION (0xB1)
#define FUSION_RECORDS           (16U)   ///< New fixes held between frames, the newest kept
#define FUSION_FRAME_MAX_BYTES   (220U)  ///< Under the radio payload once the link layer's header is added
#define FUSION_INVALID_U16       (UINT16_MAX)
#define FUSION_INVALID_I8        (INT8_MIN)
#endif

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

/// Every new solve the secure side publishes, in a ring, whatever the drive
/// mode: fix rate and jitter come from the sequence against the tick
typedef struct __attribute__((packed)) {
    uint32_t tick;      ///< Serviced tick the solve was read on
    uint32_t sequence;  ///< Fix sequence
    uint32_t x;         ///< mm, as reported, before the bounds check
    uint32_t y;         ///< mm, as reported, before the bounds check
} fix_trace_t;

#define FIX_TRACE_LENGTH (3000U)  ///< 5 minutes at 10 Hz
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
#endif

#if defined(DB_BENCH_FUSION)
/// One new fix and what the control core did with it, as the frame carries it before its lines
typedef struct __attribute__((packed)) {
    uint8_t  tick;          ///< Low 8 bits of the serviced tick the solve was read on
    uint8_t  sequence;      ///< Low 8 bits of the fix sequence
    uint8_t  flags;         ///< bit 0 fix from the lines, bit 1 fix valid, bits 2-3 estimator status, bits 4-7 lines
    uint16_t solve_x;       ///< Secure side's solve, mm, saturated
    uint16_t solve_y;       ///< Secure side's solve, mm, saturated
    int8_t   fix_dx;        ///< The core's fix minus the solve, 0.25 mm, saturated at +-127, FUSION_INVALID_I8 when invalid
    int8_t   fix_dy;        ///< The core's fix minus the solve, 0.25 mm, saturated at +-127, FUSION_INVALID_I8 when invalid
    uint16_t pose_x;        ///< Estimated photodiode after the tick, 0.1 mm, FUSION_INVALID_U16 when not tracking
    uint16_t pose_y;        ///< Estimated photodiode after the tick, 0.1 mm, FUSION_INVALID_U16 when not tracking
    int16_t  heading_cdeg;  ///< Estimated heading after the tick, 0.01 deg
} fusion_head_t;
_Static_assert(sizeof(fusion_head_t) == 15, "the frame's record layout");

typedef struct {
    fusion_head_t head;
    uint8_t       line[DB_CONTROL_LINES_MAX];        ///< station << 4 | sweep << 3 | db_pose_estimator_result_t
    int8_t        innovation[DB_CONTROL_LINES_MAX];  ///< 0.25 mm, saturated at +-127, FUSION_INVALID_I8 when not gated
} fusion_record_t;
#endif

//============================= swarmit ========================================

void swarmit_send_raw_data(const uint8_t *packet, uint8_t length);

//=========================== variables ========================================

static db_control_t *_control;

#if defined(DB_BENCH_TELEMETRY)
/// The default steering with the spin recovery, selected by a bench command
static db_steering_conf_t _steering_conf_spin;
#endif

#if defined(DB_BENCH_TRACE)
__attribute__((used)) static wheel_trace_t _trace[TRACE_LENGTH];
__attribute__((used)) static uint32_t      _trace_count = 0;
static uint32_t                            _trace_tail  = 0;

__attribute__((used)) static fix_trace_t _fix_trace[FIX_TRACE_LENGTH];
__attribute__((used)) static uint32_t    _fix_trace_count = 0;  ///< Total written; the ring index is this modulo the length
#endif

#if defined(DB_BENCH_TELEMETRY)
static telemetry_step_t _telemetry_steps[TELEMETRY_STEPS];
static uint32_t         _telemetry_step_count = 0;  ///< Steps recorded; the ring index is this modulo the length
static uint32_t         _telemetry_step_sent  = 0;  ///< Step count at the last frame
static uint32_t         _telemetry_step_tick  = 0;  ///< Serviced tick of the newest step
static telemetry_fix_t  _telemetry_fixes[TELEMETRY_FIXES];
static uint32_t         _telemetry_fix_count = 0;  ///< Solves recorded; the ring index is this modulo the length
static uint32_t         _telemetry_fix_sent  = 0;  ///< Solve count at the last frame
#endif

#if defined(DB_BENCH_FUSION)
static fusion_record_t _fusion[FUSION_RECORDS];
static uint32_t        _fusion_count   = 0;      ///< Records written; the ring index is this modulo the length
static uint32_t        _fusion_sent    = 0;      ///< Record count at the last frame
static bool            _fusion_due     = false;  ///< This tick read a new solve
static uint8_t         _fusion_lines   = 0;      ///< Floor lines staged with it
static uint32_t        _fusion_solve_x = 0;      ///< That solve, mm
static uint32_t        _fusion_solve_y = 0;      ///< That solve, mm
#endif

//=========================== private functions ================================

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_FUSION)
static void _put(uint8_t *buf, size_t *length, const void *value, size_t size) {
    memcpy(&buf[*length], value, size);
    *length += size;
}

static inline uint8_t _saturate_u8(uint32_t value) {
    return (value > UINT8_MAX) ? UINT8_MAX : (uint8_t)value;
}
#endif

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
static void _record_step(uint32_t tick, uint32_t elapsed, const db_control_input_t *in) {
    int8_t pwm_left  = _pwm_recorded(_control->pwm_left, _control->brake_left, _control->wheel_left.stalled);
    int8_t pwm_right = _pwm_recorded(_control->pwm_right, _control->brake_right, _control->wheel_right.stalled);
#if defined(DB_BENCH_TELEMETRY)
    _telemetry_steps[_telemetry_step_count % TELEMETRY_STEPS] = (telemetry_step_t){
        .counts_left    = _saturate_i8(in->counts_left),
        .counts_right   = _saturate_i8(in->counts_right),
        .pwm_left       = pwm_left,
        .pwm_right      = pwm_right,
        .setpoint_left  = _saturate_i8((int32_t)_control->wheel_left.setpoint / 10),
        .setpoint_right = _saturate_i8((int32_t)_control->wheel_right.setpoint / 10),
    };
    _telemetry_step_count++;
    _telemetry_step_tick = tick;
#endif

#if defined(DB_BENCH_TRACE)
    if (_control->drive_mode != DB_CONTROL_DRIVE_IDLE) {
        _trace_tail = TRACE_TAIL_TICKS;
    } else if (_trace_tail > 0) {
        _trace_tail--;
    } else {
        return;
    }
    if (_trace_count < TRACE_LENGTH) {
        _trace[_trace_count++] = (wheel_trace_t){
            .tick           = tick,
            .setpoint_left  = (int16_t)_control->wheel_left.setpoint,
            .setpoint_right = (int16_t)_control->wheel_right.setpoint,
            .counts_left    = (int16_t)in->counts_left,
            .counts_right   = (int16_t)in->counts_right,
            .pwm_left       = pwm_left,
            .pwm_right      = pwm_right,
            .elapsed        = (uint8_t)elapsed,
            .mode           = (uint8_t)_control->drive_mode,
        };
    }
#else
    (void)elapsed;
#endif
}
#endif

#if defined(DB_BENCH_TELEMETRY)
/// Every step and every new solve since the previous frame, newest kept when
/// there are more than a frame holds; resets the worst tick backlog it reports.
/// Layout: type, newest step tick (u32), steps carried, steps dropped, backlog,
/// drive mode in the low nibble with the steering state in the high one,
/// encoder totals (i32 x 2), solves carried, solves dropped, then the steps
/// oldest first, then the solves oldest first, then the estimator: status, the
/// last gated fix's squared distance x 10 (u16, saturated), and its kidnap and
/// re-anchor counts (u8 each, wrapping), then the steering's point index and
/// corrections made (u8 each).
static void _send_bench_telemetry(uint8_t *buf, uint32_t *max_tick_backlog) {
    size_t                     length    = 0;
    const db_pose_estimator_t *estimator = &_control->estimator;
    const db_steering_t       *steering  = &_control->steering;

    uint32_t steps     = _telemetry_step_count - _telemetry_step_sent;
    uint32_t steps_out = (steps > TELEMETRY_STEPS) ? TELEMETRY_STEPS : steps;
    uint32_t fixes     = _telemetry_fix_count - _telemetry_fix_sent;
    uint32_t fixes_out = (fixes > TELEMETRY_FIXES) ? TELEMETRY_FIXES : fixes;

    buf[length++] = DB_PROTOCOL_BENCH_TELEMETRY;
    _put(buf, &length, &_telemetry_step_tick, sizeof(_telemetry_step_tick));
    buf[length++]     = (uint8_t)steps_out;
    buf[length++]     = _saturate_u8(steps - steps_out);
    buf[length++]     = _saturate_u8(*max_tick_backlog);
    *max_tick_backlog = 0;
    buf[length++]     = (uint8_t)(_control->drive_mode | (steering->state << 4));
    _put(buf, &length, &_control->encoder_left, sizeof(_control->encoder_left));
    _put(buf, &length, &_control->encoder_right, sizeof(_control->encoder_right));
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

#if defined(DB_BENCH_FUSION)
static inline uint16_t _fusion_u16(float value) {
    return (value >= 0 && value < (float)FUSION_INVALID_U16) ? (uint16_t)lroundf(value) : FUSION_INVALID_U16;
}

/// Saturated to [-127, 127]; FUSION_INVALID_I8 only for NaN
static inline int8_t _fusion_i8(float value) {
    if (isnan(value)) {
        return FUSION_INVALID_I8;
    }
    return (value >= (float)INT8_MAX) ? INT8_MAX : ((value <= -(float)INT8_MAX) ? -INT8_MAX : (int8_t)lroundf(value));
}

/// After the tick that read a new solve: the solve, the core's fix and lines, the pose they left
static void _fusion_record(uint32_t tick) {
    if (!_fusion_due) {
        return;
    }
    _fusion_due = false;

    const db_control_fusion_record_t *core       = &_control->last_fusion;
    const db_pose_estimator_t        *est        = &_control->estimator;
    fusion_record_t                  *record     = &_fusion[_fusion_count % FUSION_RECORDS];
    uint8_t                           lines      = (_fusion_lines > 0) ? core->count : 0;
    bool                              valid      = (lines > 0) ? core->fix_valid : (_fusion_solve_x <= 100000U && _fusion_solve_y <= 100000U);
    bool                              from_lines = (lines > 0) && core->fix_from_lines;
    float                             fix_x      = (lines > 0) ? core->fix_x : (float)_fusion_solve_x;
    float                             fix_y      = (lines > 0) ? core->fix_y : (float)_fusion_solve_y;

    record->head.tick     = (uint8_t)tick;
    record->head.sequence = (uint8_t)_control->fix_sequence;
    record->head.flags    = (uint8_t)((from_lines ? 1U : 0U) | (valid ? 2U : 0U) | ((uint8_t)est->status << 2) | (lines << 4));
    record->head.solve_x  = (_fusion_solve_x > UINT16_MAX) ? UINT16_MAX : (uint16_t)_fusion_solve_x;
    record->head.solve_y  = (_fusion_solve_y > UINT16_MAX) ? UINT16_MAX : (uint16_t)_fusion_solve_y;
    record->head.fix_dx   = valid ? _fusion_i8(4.0f * (fix_x - (float)_fusion_solve_x)) : FUSION_INVALID_I8;
    record->head.fix_dy   = valid ? _fusion_i8(4.0f * (fix_y - (float)_fusion_solve_y)) : FUSION_INVALID_I8;

    float sensor_x = -1.0f;
    float sensor_y = -1.0f;
    if (est->status != DB_POSE_ESTIMATOR_TRACKING || !db_pose_estimator_sensor(est, &sensor_x, &sensor_y)) {
        sensor_x = -1.0f;
        sensor_y = -1.0f;
    }
    record->head.pose_x       = _fusion_u16(10.0f * sensor_x);
    record->head.pose_y       = _fusion_u16(10.0f * sensor_y);
    record->head.heading_cdeg = (int16_t)lroundf(est->theta * 18000.0f / (float)M_PI);
    for (uint8_t i = 0; i < lines; i++) {
        record->line[i]       = (uint8_t)(((core->station_sweep[i] >> 1) << 4) | ((core->station_sweep[i] & 1U) << 3) | (core->result[i] & 7U));
        record->innovation[i] = _fusion_i8(4.0f * core->innovation_mm[i]);
    }
    _fusion_count++;
}

/// Every new fix since the previous frame that fits, the newest kept.
/// Layout: type, serviced tick at sending (u32), records carried, records dropped,
/// then the records oldest first, each 15 bytes then 2 per line (line byte,
/// innovation): tick (u8), sequence (u8), flags, solve x and y (u16 mm),
/// fix minus solve (i8 x 2, 0.25 mm), photodiode x and y (u16 0.1 mm),
/// heading (i16 0.01 deg).
static void _send_bench_fusion(uint32_t tick, uint8_t *buf) {
    size_t length = 0;

    uint32_t pending = _fusion_count - _fusion_sent;
    if (pending == 0) {
        return;
    }
    if (pending > FUSION_RECORDS) {
        pending = FUSION_RECORDS;
    }
    // From the newest back, as many as fit
    size_t   bytes = 7;
    uint32_t out   = 0;
    while (out < pending) {
        const fusion_record_t *r    = &_fusion[(_fusion_count - 1U - out) % FUSION_RECORDS];
        size_t                 size = sizeof(fusion_head_t) + 2U * (r->head.flags >> 4);
        if (bytes + size > FUSION_FRAME_MAX_BYTES) {
            break;
        }
        bytes += size;
        out++;
    }

    buf[length++] = DB_PROTOCOL_BENCH_FUSION;
    _put(buf, &length, &tick, sizeof(tick));
    buf[length++] = (uint8_t)out;
    buf[length++] = _saturate_u8(_fusion_count - _fusion_sent - out);
    for (uint32_t i = _fusion_count - out; i != _fusion_count; i++) {
        const fusion_record_t *r = &_fusion[i % FUSION_RECORDS];
        _put(buf, &length, &r->head, sizeof(r->head));
        for (uint8_t l = 0; l < (r->head.flags >> 4); l++) {
            buf[length++] = r->line[l];
            buf[length++] = (uint8_t)r->innovation[l];
        }
    }
    _fusion_sent = _fusion_count;
    swarmit_send_raw_data(buf, (uint8_t)length);
}
#endif

//=========================== public ===========================================

void bench_init(db_control_t *control) {
    _control = control;
#if defined(DB_BENCH_TELEMETRY)
    _steering_conf_spin         = control->conf->steering;
    _steering_conf_spin.recover = DB_STEERING_RECOVER_SPIN;
#endif
}

/// With DB_BENCH_TELEMETRY, a waypoint threshold of 0xFFFF drops the
/// estimator's pose, as a kidnap does, to exercise the steering's heading
/// recovery; 0xFFFE does the same with the spin recovery, until the next waypoint.
bool bench_rx(const uint8_t *packet, size_t length) {
#if defined(DB_BENCH_TELEMETRY)
    if (packet[0] != DB_PROTOCOL_LH2_WAYPOINTS) {
        return false;
    }
    db_steering_path_t path;
    uint8_t            batch_id;
    if (!db_steering_path_from_wire(&packet[1], length - 1, &path, &batch_id)) {
        return false;
    }
    if (path.threshold_mm >= (float)(UINT16_MAX - 1)) {
        _control->steering.conf = (path.threshold_mm == (float)UINT16_MAX) ? &_control->conf->steering : &_steering_conf_spin;
        db_pose_estimator_init(&_control->estimator, &_control->conf->estimator);
        return true;
    }
    _control->steering.conf = &_control->conf->steering;
    return false;
#else
    (void)packet;
    (void)length;
    return false;
#endif
}

void bench_rx_applied(db_control_drive_mode_t previous_mode) {
#if defined(DB_BENCH_TRACE)
    if (previous_mode == DB_CONTROL_DRIVE_IDLE && _control->drive_mode != DB_CONTROL_DRIVE_IDLE) {
        _trace_count = 0;
    }
#else
    (void)previous_mode;
#endif
}

void bench_fix(uint32_t tick, uint32_t sequence, uint32_t x, uint32_t y, uint8_t lines) {
#if defined(DB_BENCH_FUSION)
    _fusion_due     = sequence != _control->fix_sequence;
    _fusion_lines   = lines;
    _fusion_solve_x = x;
    _fusion_solve_y = y;
#else
    (void)lines;
#endif

#if defined(DB_BENCH_TRACE) || defined(DB_BENCH_TELEMETRY)
    // An unchanged sequence is the previous solve read a second time
    if (sequence == _control->fix_sequence) {
        return;
    }
#else
    (void)tick;
#endif
#if defined(DB_BENCH_TRACE)
    _fix_trace[_fix_trace_count % FIX_TRACE_LENGTH] = (fix_trace_t){
        .tick     = tick,
        .sequence = sequence,
        .x        = x,
        .y        = y,
    };
    _fix_trace_count++;
#endif
#if defined(DB_BENCH_TELEMETRY)
    _telemetry_fixes[_telemetry_fix_count % TELEMETRY_FIXES] = (telemetry_fix_t){
        .tick     = (uint16_t)tick,
        .sequence = (uint16_t)sequence,
        .x        = (x > UINT16_MAX) ? UINT16_MAX : (uint16_t)x,
        .y        = (y > UINT16_MAX) ? UINT16_MAX : (uint16_t)y,
    };
    _telemetry_fix_count++;
#endif
}

void bench_tick(uint32_t tick, uint32_t elapsed, const db_control_input_t *in) {
#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE)
    _record_step(tick, elapsed, in);
#else
    (void)elapsed;
    (void)in;
#endif
#if defined(DB_BENCH_FUSION)
    _fusion_record(tick);
#else
    (void)tick;
#endif
}

uint32_t bench_frames(void) {
    uint32_t frames = 0;
#if defined(DB_BENCH_TELEMETRY)
    frames++;
#endif
#if defined(DB_BENCH_FUSION)
    frames++;
#endif
    return frames;
}

void bench_advertise(uint32_t tick, uint8_t *buffer, uint32_t *max_tick_backlog) {
#if defined(DB_BENCH_TELEMETRY)
    _send_bench_telemetry(buffer, max_tick_backlog);
#else
    (void)max_tick_backlog;
#endif
#if defined(DB_BENCH_FUSION)
    _send_bench_fusion(tick, buffer);
#else
    (void)tick;
    (void)buffer;
#endif
}

#endif
