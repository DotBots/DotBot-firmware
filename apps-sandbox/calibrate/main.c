/**
 * @file
 * @defgroup project_calibrate    LH2 calibration capture app
 * @ingroup projects
 * @brief Captures raw LH2 counts when the robot's own button is pressed
 *
 * A press on the user button (P1.09) runs a countdown on mcu-led3 (P1.06),
 * then takes CAPTURE_READS raw-count reads per visible station with the LED
 * solid. A still capture is acknowledged with three blinks and sent three
 * times as log events, paced to half the node's uplink budget; a moving one
 * is refused with a slow pulse and not sent.
 *
 * @copyright Inria, 2026
 */

#include <nrf.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV              (0)
#define TICK_MS                (10U)
#define KEEP_ALIVE_TICKS       (10U)       ///< Watchdog reload period outside the capture window
#define CAPTURE_READS          (25U)       ///< Reads per station in one capture
#define STATIONS_MAX           (4U)        ///< Stations kept per capture; 4 x 25 records fit 8 chunks
#define WINDOW_PRESENCE_MS     (100U)      ///< A station seen within this much of the window is visible
#define WINDOW_TIMEOUT_MS      (800U)      ///< Under the ~1 s watchdog, with no keep_alive inside
#define STILLNESS_SPREAD_MAX   (40U)       ///< Largest max - min of a station's counts over the reads
#define COUNT_MAX              (1U << 17)  ///< Counts are indexes into a 17-bit LFSR sequence
#define COPIES                 (3U)        ///< Each capture is sent this many times
#define SEND_SPACING_TICKS     (2U)        ///< Least between two log events, so the net core drains the first
#define UNJOINED_SPACING_TICKS (100U)      ///< Between two log events while the uplink budget reads 0
#define BUDGET_SHARE_PCT       (50U)       ///< Share of the uplink budget a capture's log events may use
#define BUDGET_SCALE           (100U)      ///< The uplink budget is in packets per second x 100
#define PCT                    (100U)
#define MS_PER_S               (1000U)
#define RECORD_SIZE            (9U)                                 ///< [lh_index:1][count1:4 LE][count2:4 LE]
#define LOG_SIZE_MAX           (127U)                               ///< swarmit_log_data refuses anything longer
#define CAPTURE_TAG            (0xCBU)                              ///< First byte of a button capture's log event
#define CHUNK_RECORDS          ((LOG_SIZE_MAX - 2U) / RECORD_SIZE)  ///< 13
#define BLINK_PERIOD_MS        (330U)                               ///< One blink of the countdown and the acknowledgement
#define BLINK_ON_MS            (100U)
#define BLINK_COUNT            (3U)
#define BLINK_SEQUENCE_MS      (BLINK_COUNT * BLINK_PERIOD_MS)
#define REFUSE_PULSE_MS        (1000U)  ///< One on/off pulse of the refusal
#define REFUSE_DURATION_MS     (2000U)

// A log event is [CAPTURE_TAG][header][records], the header being ppppp ccc:
// p = press counter mod 32, c = chunk index. The last chunk of a capture
// carries fewer than CHUNK_RECORDS records, so a capture whose record count is
// a multiple of CHUNK_RECORDS ends with an empty chunk. PyDotBot's
// dotbot/tests/lh2_button_fixture.py pins this layout; change both together.
_Static_assert(STATIONS_MAX *CAPTURE_READS / CHUNK_RECORDS < 8, "chunk index must fit 3 bits");

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
    STATE_IDLE,
    STATE_COUNTDOWN,
    STATE_WINDOW,
    STATE_SEND,
    STATE_CAPTURED,
    STATE_REFUSED,
} capture_state_t;

//============================= swarmit ========================================

typedef void (*ipc_isr_cb_t)(const uint8_t *, size_t);

void     swarmit_keep_alive(void);
void     swarmit_ipc_isr(ipc_isr_cb_t cb);
void     swarmit_log_data(uint8_t *data, size_t length);
uint16_t swarmit_get_uplink_budget(void);
uint8_t  swarmit_localization_get_raw_counts(lh2_raw_sample_t *samples, uint8_t max);
void     swarmit_localization_handle_isr(void);

//=========================== variables ========================================

static const gpio_t _button = { .port = 1, .pin = 9 };  ///< mcu-sw, pulled up, low when pressed
static const gpio_t _led    = { .port = 1, .pin = 6 };  ///< mcu-led3, on when low

static volatile uint32_t _tick_count = 0;
static volatile bool     _pressed    = false;

static struct {
    capture_state_t  state;
    uint32_t         state_tick;  ///< Tick the current state started at
    uint8_t          press_counter;
    uint8_t          station_count;
    uint8_t          stations[STATIONS_MAX];
    uint8_t          reads[STATIONS_MAX];
    lh2_raw_sample_t samples[STATIONS_MAX][CAPTURE_READS];
    uint8_t          copy;
    uint8_t          chunk;
    uint32_t         next_send_tick;
    uint32_t         keep_alive_tick;  ///< Tick of the last watchdog reload
} _app;

static lh2_raw_sample_t _drain[16];
static uint8_t __attribute__((aligned(4))) _log[LOG_SIZE_MAX];

//=========================== private ==========================================

static void _led_set(bool on) {
    if (on) {
        db_gpio_clear(&_led);
    } else {
        db_gpio_set(&_led);
    }
}

static void _tick(void) {
    _tick_count++;
}

static void _on_press(void *ctx) {
    (void)ctx;
    _pressed = true;
}

static void _enter(capture_state_t state) {
    _app.state      = state;
    _app.state_tick = _tick_count;
}

static void _keep_alive(void) {
    swarmit_keep_alive();
    _app.keep_alive_tick = _tick_count;
}

// Elapsed rather than a multiple of the tick: a backlog serviced at once can step over a multiple
static void _keep_alive_if_due(void) {
    if (_tick_count - _app.keep_alive_tick >= KEEP_ALIVE_TICKS) {
        _keep_alive();
    }
}

static uint32_t _elapsed_ms(void) {
    return (_tick_count - _app.state_tick) * TICK_MS;
}

static int _station_slot(uint8_t lh_index) {
    for (uint8_t i = 0; i < _app.station_count; i++) {
        if (_app.stations[i] == lh_index) {
            return i;
        }
    }
    return -1;
}

static void _window_poll(void) {
    uint8_t n = swarmit_localization_get_raw_counts(_drain, sizeof(_drain) / sizeof(_drain[0]));
    for (uint8_t i = 0; i < n; i++) {
        int slot = _station_slot(_drain[i].lh_index);
        if (slot < 0) {
            if (_elapsed_ms() > WINDOW_PRESENCE_MS || _app.station_count >= STATIONS_MAX) {
                continue;
            }
            slot                = _app.station_count++;
            _app.stations[slot] = _drain[i].lh_index;
            _app.reads[slot]    = 0;
        }
        if (_app.reads[slot] < CAPTURE_READS) {
            _app.samples[slot][_app.reads[slot]++] = _drain[i];
        }
    }
}

static bool _window_full(void) {
    if (_app.station_count == 0 || _elapsed_ms() <= WINDOW_PRESENCE_MS) {
        return false;
    }
    for (uint8_t s = 0; s < _app.station_count; s++) {
        if (_app.reads[s] < CAPTURE_READS) {
            return false;
        }
    }
    return true;
}

/// The larger of the two sweeps' max - min over a station's reads
static uint32_t _spread(const lh2_raw_sample_t *samples) {
    uint32_t lo1 = UINT32_MAX, hi1 = 0, lo2 = UINT32_MAX, hi2 = 0;
    for (uint8_t r = 0; r < CAPTURE_READS; r++) {
        lo1 = samples[r].count1 < lo1 ? samples[r].count1 : lo1;
        hi1 = samples[r].count1 > hi1 ? samples[r].count1 : hi1;
        lo2 = samples[r].count2 < lo2 ? samples[r].count2 : lo2;
        hi2 = samples[r].count2 > hi2 ? samples[r].count2 : hi2;
    }
    return hi1 - lo1 > hi2 - lo2 ? hi1 - lo1 : hi2 - lo2;
}

static bool _capture_clean(void) {
    if (!_window_full()) {
        return false;
    }
    for (uint8_t s = 0; s < _app.station_count; s++) {
        for (uint8_t r = 0; r < CAPTURE_READS; r++) {
            const lh2_raw_sample_t *sample = &_app.samples[s][r];
            if (sample->count1 == 0 || sample->count2 == 0 || sample->count1 == sample->count2 ||
                sample->count1 >= COUNT_MAX || sample->count2 >= COUNT_MAX) {
                return false;
            }
        }
        if (_spread(_app.samples[s]) > STILLNESS_SPREAD_MAX) {
            return false;
        }
    }
    return true;
}

static uint8_t _chunk_count(void) {
    return (uint8_t)((_app.station_count * CAPTURE_READS) / CHUNK_RECORDS + 1);
}

static size_t _put_u32_le(uint8_t *buf, size_t at, uint32_t value) {
    for (uint8_t shift = 0; shift < 32; shift += 8) {
        buf[at++] = (uint8_t)(value >> shift);
    }
    return at;
}

// Records go out read by read, each read listing every station in the same order
static void _send_chunk(uint8_t chunk) {
    uint32_t total  = (uint32_t)_app.station_count * CAPTURE_READS;
    uint32_t first  = (uint32_t)chunk * CHUNK_RECORDS;
    uint32_t last   = first + CHUNK_RECORDS < total ? first + CHUNK_RECORDS : total;
    size_t   length = 0;
    _log[length++]  = CAPTURE_TAG;
    _log[length++]  = (uint8_t)(((_app.press_counter & 0x1FU) << 3) | (chunk & 0x07U));
    for (uint32_t k = first; k < last; k++) {
        const lh2_raw_sample_t *sample = &_app.samples[k % _app.station_count][k / _app.station_count];
        _log[length++]                 = sample->lh_index;
        length                         = _put_u32_le(_log, length, sample->count1);
        length                         = _put_u32_le(_log, length, sample->count2);
    }
    swarmit_log_data(_log, length);
}

static bool _blinks_on(uint32_t ms) {
    return ms < BLINK_SEQUENCE_MS && (ms % BLINK_PERIOD_MS) < BLINK_ON_MS;
}

// The budget can change with the schedule, so it is read before every event
static uint32_t _send_spacing_ticks(void) {
    uint32_t budget_cpps = swarmit_get_uplink_budget();
    if (budget_cpps == 0) {
        return UNJOINED_SPACING_TICKS;
    }
    // Ticks between events at BUDGET_SHARE_PCT of the budget, rounded up
    uint32_t share   = budget_cpps * BUDGET_SHARE_PCT * TICK_MS;
    uint32_t spacing = (BUDGET_SCALE * PCT * MS_PER_S + share - 1U) / share;
    return spacing < SEND_SPACING_TICKS ? SEND_SPACING_TICKS : spacing;
}

static void _service(void) {
    uint32_t ms = _elapsed_ms();
    switch (_app.state) {
        case STATE_IDLE:
            _keep_alive_if_due();
            if (_pressed) {
                _enter(STATE_COUNTDOWN);
            }
            break;
        case STATE_COUNTDOWN:
            _keep_alive_if_due();
            if (ms < BLINK_SEQUENCE_MS) {
                _led_set(_blinks_on(ms));
                break;
            }
            // The last reload before the window: keep_alive drains the counts the window reads
            _keep_alive();
            memset(_app.reads, 0, sizeof(_app.reads));
            _app.station_count = 0;
            swarmit_localization_get_raw_counts(_drain, sizeof(_drain) / sizeof(_drain[0]));
            _led_set(true);
            _enter(STATE_WINDOW);
            break;
        case STATE_WINDOW:
            if (!_window_full() && ms < WINDOW_TIMEOUT_MS) {
                break;
            }
            _keep_alive();
            _led_set(false);
            if (_capture_clean()) {
                _app.copy           = 0;
                _app.chunk          = 0;
                _app.next_send_tick = _tick_count;
                _enter(STATE_SEND);
            } else {
                _enter(STATE_REFUSED);
            }
            break;
        case STATE_SEND:
            _keep_alive_if_due();
            _led_set(_blinks_on(ms));
            if ((int32_t)(_tick_count - _app.next_send_tick) < 0) {
                break;
            }
            _send_chunk(_app.chunk);
            _app.next_send_tick = _tick_count + _send_spacing_ticks();
            if (++_app.chunk == _chunk_count()) {
                _app.chunk = 0;
                if (++_app.copy == COPIES) {
                    _app.press_counter++;
                    _app.state = STATE_CAPTURED;  // keeps state_tick: the blinks run on
                }
            }
            break;
        case STATE_CAPTURED:
            _keep_alive_if_due();
            if (ms < BLINK_SEQUENCE_MS) {
                _led_set(_blinks_on(ms));
                break;
            }
            _led_set(false);
            _pressed = false;
            _enter(STATE_IDLE);
            break;
        case STATE_REFUSED:
            _keep_alive_if_due();
            if (ms < REFUSE_DURATION_MS) {
                _led_set((ms % REFUSE_PULSE_MS) < REFUSE_PULSE_MS / 2U);
                break;
            }
            _led_set(false);
            _pressed = false;
            _enter(STATE_IDLE);
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
    _keep_alive();

    db_gpio_init(&_led, DB_GPIO_OUT);
    _led_set(false);
    db_gpio_init_irq(&_button, DB_GPIO_IN, DB_GPIO_IRQ_EDGE_FALLING, _on_press, NULL);

    _enter(STATE_IDLE);
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, TICK_MS, &_tick);

    uint32_t serviced = _tick_count;
    while (1) {
        __WFE();
        if (_app.state == STATE_WINDOW) {
            _window_poll();
        }
        while (serviced != _tick_count) {
            serviced++;
            _service();
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
