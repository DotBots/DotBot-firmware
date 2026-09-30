/**
 * @file
 * @defgroup project_calibrate_spin    LH2 calibration spin app
 * @ingroup projects
 * @brief Spins in place and sends the raw LH2 counts the photodiode read
 *
 * After a countdown on the LED the robot turns counter clockwise in place for
 * SPIN_DEG on the wheel speed loop, keeping one raw-count read per visible
 * station every RECORD_SPACING_MS. A spin in place turns about the axle
 * midpoint, so the reads trace a circle around it on the floor.
 * Once the robot stands, the reads are sent COPIES times as log events paced
 * to the node's minimum TX interval, then the LED blinks slowly until the app
 * is stopped.
 *
 * @copyright Inria, 2026
 */

#include <nrf.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "motors.h"
#include "qdec.h"
#include "timer.h"
#include "wheel_control.h"

//=========================== defines ==========================================

#define TIMER_DEV              (0)
#define QDEC_LEFT              (0)
#define QDEC_RIGHT             (1)
#define TICK_MS                (DB_WHEEL_CONTROL_TICK_MS)
#define KEEP_ALIVE_TICKS       (20U)      ///< 200 ms, under the ~1 s watchdog
#define COUNTDOWN_TICKS        (300U)     ///< 3 s of fast blinks before the robot moves
#define SETTLE_TICKS           (50U)      ///< 500 ms standing before the reads go out
#define SPIN_DEG               (-720.0f)  ///< Two turns, negative is counter clockwise
#define SPIN_MM_S              (60.0f)    ///< Wheel speed of the spin
#define RECORD_SPACING_MS      (30U)      ///< Least between two kept reads of one station
#define STATIONS_MAX           (4U)
#define READS_MAX              (320U)      ///< Per station: 9.6 s at RECORD_SPACING_MS
#define COUNT_MAX              (1U << 17)  ///< Counts are indexes into a 17-bit LFSR sequence
#define COPIES                 (2U)        ///< Each read is sent this many times
#define SEND_SPACING_TICKS     (2U)        ///< Least between two log events, so the net core drains the first
#define UNJOINED_SPACING_TICKS (100U)      ///< Between two log events while the minimum TX interval reads 0
#define US_PER_TICK            (TICK_MS * 1000U)
#define RECORD_SIZE            (9U)                                          ///< [lh_index:1][count1:4 LE][count2:4 LE]
#define LOG_SIZE_MAX           (127U)                                        ///< swarmit_log_data refuses anything longer
#define SPIN_TAG               (0xCCU)                                       ///< First byte of a spin's log event
#define HEADER_SIZE            (4U)                                          ///< [tag][run][chunk][chunks]
#define CHUNK_RECORDS          ((LOG_SIZE_MAX - HEADER_SIZE) / RECORD_SIZE)  ///< 13
#define BLINK_FAST_TICKS       (10U)
#define BLINK_SLOW_TICKS       (50U)

// A log event is [SPIN_TAG][run][chunk][chunks][records]: run tells two spins of
// one robot apart, chunk is the index and chunks the total. Records are the
// reads of the first station in the order they were taken, then the next
// station's. PyDotBot's dotbot/calibration/spin.py decodes this layout; change
// both together.
_Static_assert(STATIONS_MAX *READS_MAX / CHUNK_RECORDS < 255, "chunk count must fit a byte");

typedef struct {
    uint32_t count1;
    uint32_t count2;
    uint8_t  lh_index;
    uint8_t  _pad[3];
} lh2_raw_sample_t;
_Static_assert(sizeof(lh2_raw_sample_t) == 12, "must match the bootloader's layout");
_Static_assert(offsetof(lh2_raw_sample_t, count1) == 0, "must match the bootloader's layout");
_Static_assert(offsetof(lh2_raw_sample_t, count2) == 4, "must match the bootloader's layout");
_Static_assert(offsetof(lh2_raw_sample_t, lh_index) == 8, "must match the bootloader's layout");

typedef enum {
    STATE_COUNTDOWN,
    STATE_SPIN,
    STATE_SETTLE,
    STATE_SEND,
    STATE_DONE,
} spin_state_t;

//============================= swarmit ========================================

typedef void (*ipc_isr_cb_t)(const uint8_t *, size_t);

void     swarmit_keep_alive(void);
void     swarmit_ipc_isr(ipc_isr_cb_t cb);
void     swarmit_log_data(uint8_t *data, size_t length);
uint32_t swarmit_get_min_tx_interval_us(void);
uint8_t  swarmit_localization_get_raw_counts(lh2_raw_sample_t *samples, uint8_t max);
void     swarmit_localization_handle_isr(void);

//=========================== variables ========================================

static const qdec_conf_t _qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t _qdec_right = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

/// Keep in step with the gains of apps-sandbox/dotbot
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

static volatile uint32_t _tick_count = 0;

static struct {
    spin_state_t       state;
    uint32_t           state_tick;
    uint32_t           keep_alive_tick;
    db_wheel_control_t wheel_left;
    db_wheel_control_t wheel_right;
    db_wheel_goal_t    goal;
    uint8_t            run;
    bool               run_set;
    uint8_t            station_count;
    uint8_t            stations[STATIONS_MAX];
    uint16_t           reads[STATIONS_MAX];
    uint32_t           last_read_tick[STATIONS_MAX];
    lh2_raw_sample_t   samples[STATIONS_MAX][READS_MAX];
    uint8_t            copy;
    uint8_t            chunk;
    uint32_t           next_send_tick;
} _app;

static lh2_raw_sample_t _drain[16];
static uint8_t          _log[LOG_SIZE_MAX] __attribute__((aligned(4)));

//=========================== private ==========================================

static void _tick(void) {
    _tick_count++;
}

static void _enter(spin_state_t state) {
    _app.state      = state;
    _app.state_tick = _tick_count;
}

static uint32_t _in_state(void) {
    return _tick_count - _app.state_tick;
}

static bool _valid(const lh2_raw_sample_t *sample) {
    return sample->count1 != 0 && sample->count2 != 0 && sample->count1 != sample->count2 &&
           sample->count1 < COUNT_MAX && sample->count2 < COUNT_MAX;
}

static int _station_slot(uint8_t lh_index) {
    for (uint8_t i = 0; i < _app.station_count; i++) {
        if (_app.stations[i] == lh_index) {
            return i;
        }
    }
    if (_app.station_count >= STATIONS_MAX) {
        return -1;
    }
    _app.stations[_app.station_count] = lh_index;
    _app.reads[_app.station_count]    = 0;
    return _app.station_count++;
}

/// Takes the newest read of each station; the bootloader keeps one per station, not a queue
static void _record(void) {
    uint8_t n = swarmit_localization_get_raw_counts(_drain, sizeof(_drain) / sizeof(_drain[0]));
    for (uint8_t i = 0; i < n; i++) {
        if (!_valid(&_drain[i])) {
            continue;
        }
        if (!_app.run_set) {
            // Any byte that differs between two spins of one robot will do
            _app.run     = (uint8_t)_drain[i].count1;
            _app.run_set = true;
        }
        int slot = _station_slot(_drain[i].lh_index);
        if (slot < 0 || _app.reads[slot] >= READS_MAX) {
            continue;
        }
        if (_app.reads[slot] > 0 && (_tick_count - _app.last_read_tick[slot]) * TICK_MS < RECORD_SPACING_MS) {
            continue;
        }
        _app.samples[slot][_app.reads[slot]++] = _drain[i];
        _app.last_read_tick[slot]              = _tick_count;
    }
}

/// The solve inside keep_alive consumes a read, so the app takes its own first
static void _keep_alive(bool recording) {
    if (recording) {
        _record();
    }
    swarmit_keep_alive();
    _app.keep_alive_tick = _tick_count;
}

static void _keep_alive_if_due(bool recording) {
    if (_tick_count - _app.keep_alive_tick >= KEEP_ALIVE_TICKS) {
        _keep_alive(recording);
    }
}

/// One step of the speed loop; true once the goal is done and both wheels stand
static bool _drive(uint32_t elapsed) {
    uint32_t dbl_left;
    uint32_t dbl_right;
    int32_t  left  = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_LEFT, &dbl_left), dbl_left);
    int32_t  right = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_RIGHT, &dbl_right), dbl_right);
    if (_app.wheel_left.stalled || _app.wheel_right.stalled) {
        db_wheel_goal_start(&_app.goal, 0, 0, 0);
    }
    float setpoint_left;
    float setpoint_right;
    bool  driving = db_wheel_goal_step(&_app.goal, left, right, &setpoint_left, &setpoint_right);
    db_wheel_control_set_setpoint(&_app.wheel_left, setpoint_left);
    db_wheel_control_set_setpoint(&_app.wheel_right, setpoint_right);
    int8_t pwm_left  = db_wheel_control_step(&_app.wheel_left, left, elapsed);
    int8_t pwm_right = db_wheel_control_step(&_app.wheel_right, right, elapsed);
    db_motors_set_pwm_brake(pwm_left, pwm_right, _app.wheel_left.brake, _app.wheel_right.brake);
    return !driving && !_app.wheel_left.brake && !_app.wheel_right.brake;
}

static uint16_t _record_total(void) {
    uint16_t total = 0;
    for (uint8_t s = 0; s < _app.station_count; s++) {
        total += _app.reads[s];
    }
    return total;
}

static uint8_t _chunk_count(void) {
    return (uint8_t)((_record_total() + CHUNK_RECORDS - 1U) / CHUNK_RECORDS);
}

/// The k-th record across the stations, station by station
static const lh2_raw_sample_t *_record_at(uint16_t k) {
    for (uint8_t s = 0; s < _app.station_count; s++) {
        if (k < _app.reads[s]) {
            return &_app.samples[s][k];
        }
        k -= _app.reads[s];
    }
    return NULL;
}

static size_t _put_u32_le(uint8_t *buf, size_t at, uint32_t value) {
    for (uint8_t shift = 0; shift < 32; shift += 8) {
        buf[at++] = (uint8_t)(value >> shift);
    }
    return at;
}

static void _send_chunk(uint8_t chunk) {
    uint16_t total  = _record_total();
    uint16_t first  = (uint16_t)chunk * CHUNK_RECORDS;
    uint16_t last   = first + CHUNK_RECORDS < total ? first + CHUNK_RECORDS : total;
    size_t   length = 0;
    _log[length++]  = SPIN_TAG;
    _log[length++]  = _app.run;
    _log[length++]  = chunk;
    _log[length++]  = _chunk_count();
    for (uint16_t k = first; k < last; k++) {
        const lh2_raw_sample_t *sample = _record_at(k);
        _log[length++]                 = sample->lh_index;
        length                         = _put_u32_le(_log, length, sample->count1);
        length                         = _put_u32_le(_log, length, sample->count2);
    }
    swarmit_log_data(_log, length);
}

// The interval can change with the schedule, so it is read before every event
static uint32_t _send_spacing_ticks(void) {
    uint32_t min_tx_interval_us = swarmit_get_min_tx_interval_us();
    if (min_tx_interval_us == 0) {
        return UNJOINED_SPACING_TICKS;
    }
    uint32_t spacing = (min_tx_interval_us + US_PER_TICK - 1U) / US_PER_TICK;
    return spacing < SEND_SPACING_TICKS ? SEND_SPACING_TICKS : spacing;
}

static void _blink(uint32_t period_ticks) {
    if (_in_state() % period_ticks == 0) {
        db_gpio_toggle(&db_led1);
    }
}

static void _service(uint32_t elapsed) {
    switch (_app.state) {
        case STATE_COUNTDOWN:
            _keep_alive_if_due(false);
            _blink(BLINK_FAST_TICKS);
            if (_in_state() >= COUNTDOWN_TICKS) {
                db_gpio_set(&db_led1);
                db_wheel_goal_turn(&_app.goal, SPIN_DEG, SPIN_MM_S);
                _enter(STATE_SPIN);
            }
            break;
        case STATE_SPIN:
            _keep_alive_if_due(true);
            if (_drive(elapsed)) {
                _enter(STATE_SETTLE);
            }
            break;
        case STATE_SETTLE:
            _keep_alive_if_due(false);
            _drive(elapsed);
            if (_in_state() >= SETTLE_TICKS) {
                db_motors_coast();
                _app.copy           = 0;
                _app.chunk          = 0;
                _app.next_send_tick = _tick_count;
                _enter(_record_total() > 0 ? STATE_SEND : STATE_DONE);
            }
            break;
        case STATE_SEND:
            _keep_alive_if_due(false);
            _blink(BLINK_FAST_TICKS);
            if ((int32_t)(_tick_count - _app.next_send_tick) < 0) {
                break;
            }
            _send_chunk(_app.chunk);
            _app.next_send_tick = _tick_count + _send_spacing_ticks();
            if (++_app.chunk == _chunk_count()) {
                _app.chunk = 0;
                if (++_app.copy == COPIES) {
                    _enter(STATE_DONE);
                }
            }
            break;
        case STATE_DONE:
            _keep_alive_if_due(false);
            _blink(BLINK_SLOW_TICKS);
            break;
    }
}

static void _rx(const uint8_t *pkt, size_t len) {
    (void)pkt;
    (void)len;
}

//=========================== main =============================================

int main(void) {
    db_board_init();
    swarmit_keep_alive();

    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_motors_init();
    db_qdec_init(QDEC_LEFT, &_qdec_left, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right, NULL, NULL);
    db_wheel_control_init(&_app.wheel_left, &_wheel_conf);
    db_wheel_control_init(&_app.wheel_right, &_wheel_conf);

    _enter(STATE_COUNTDOWN);
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, TICK_MS, &_tick);

    uint32_t serviced = _tick_count;
    while (1) {
        __WFE();
        if (_app.state == STATE_SPIN) {
            _record();
        }
        uint32_t now = _tick_count;
        if (now != serviced) {
            uint32_t elapsed = now - serviced;
            serviced         = now;
            _service(elapsed);
        }
    }
}

//=========================== interrupts =======================================

void IPC_IRQHandler(void) {
    swarmit_ipc_isr(_rx);
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
