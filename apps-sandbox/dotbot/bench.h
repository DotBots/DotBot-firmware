#ifndef __BENCH_H
#define __BENCH_H

/**
 * @file
 * @ingroup project_sandbox_dotbot
 * @brief Bench-only recorders of the sandbox DotBot app, each behind its own
 * flag: DB_BENCH_TELEMETRY (a telemetry frame behind each advertisement),
 * DB_BENCH_TRACE (wheel and fix traces read over the debugger) and
 * DB_BENCH_FUSION (a fusion frame behind each advertisement). Without any of
 * them every hook is an empty inline function.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dotbot_control.h"

#if defined(DB_BENCH_TELEMETRY) || defined(DB_BENCH_TRACE) || defined(DB_BENCH_FUSION)
#define DB_BENCH
#endif

#if defined(DB_BENCH)

/// After db_control_init(); the recorders read the core through control
void bench_init(db_control_t *control);

/// Before the core applies a command; true when it is a bench-only command, which the core must not see
bool bench_rx(const uint8_t *packet, size_t length);

/// After the core applied a command, with the drive mode it had before
void bench_rx_applied(db_control_drive_mode_t previous_mode);

/// After each fix read, with the solve as read and the floor lines staged with it
void bench_fix(uint32_t tick, uint32_t sequence, uint32_t x, uint32_t y, uint8_t lines);

/// After each control tick
void bench_tick(uint32_t tick, uint32_t elapsed, const db_control_input_t *in);

/// Bench frames sent behind each advertisement
uint32_t bench_frames(void);

/// After each advertisement: sends the bench frames from buffer, and resets the backlog the telemetry reports
void bench_advertise(uint32_t tick, uint8_t *buffer, uint32_t *max_tick_backlog);

#else

static inline void bench_init(db_control_t *control) {
    (void)control;
}

static inline bool bench_rx(const uint8_t *packet, size_t length) {
    (void)packet;
    (void)length;
    return false;
}

static inline void bench_rx_applied(db_control_drive_mode_t previous_mode) {
    (void)previous_mode;
}

static inline void bench_fix(uint32_t tick, uint32_t sequence, uint32_t x, uint32_t y, uint8_t lines) {
    (void)tick;
    (void)sequence;
    (void)x;
    (void)y;
    (void)lines;
}

static inline void bench_tick(uint32_t tick, uint32_t elapsed, const db_control_input_t *in) {
    (void)tick;
    (void)elapsed;
    (void)in;
}

static inline uint32_t bench_frames(void) {
    return 0;
}

static inline void bench_advertise(uint32_t tick, uint8_t *buffer, uint32_t *max_tick_backlog) {
    (void)tick;
    (void)buffer;
    (void)max_tick_backlog;
}

#endif

#endif
