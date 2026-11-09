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
#include "bench.h"
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

/// Commands arrive in the IPC interrupt and are applied on the next tick, so
/// the main loop is the only caller of the control core.
static uint8_t       _rx_buffer[DB_CONTROL_RX_MAX_BYTES];
static size_t        _rx_length  = 0;
static volatile bool _rx_pending = false;

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
    bench_init(&_control);
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
    bench_tick(tick, elapsed, &in);
    if (out.advertise) {
        _advertise();
    }
}

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

    if (bench_rx(packet, length)) {
        return;
    }
    db_control_drive_mode_t mode = _control.drive_mode;
    db_control_rx(&_control, packet, length);
    bench_rx_applied(mode);
}

/// From the node's minimum TX interval, so a gateway on another schedule changes
/// the rate within one period
static void _advert_period_update(void) {
    db_control_set_min_tx_interval(&_control, swarmit_get_min_tx_interval_us());
    // Each bench frame is sent behind the advert, so the period stretches by one advert per frame
    _control.advert_period_ticks *= 1U + bench_frames();
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
    uint8_t             line_count = swarmit_localization_get_lines(lines, DB_CONTROL_LINES_MAX);
    db_control_lines(&_control, lines, line_count);
    bench_fix(_tick_serviced, in->fix_sequence, solve.x, solve.y, line_count);
}

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

    bench_advertise(_tick_serviced, _vars.radio_buffer, &_vars.max_tick_backlog);
}

void IPC_IRQHandler(void) {
    swarmit_ipc_isr(_rx_data_callback);
}

void SPIM4_IRQHandler(void) {
    swarmit_localization_handle_isr();
}
