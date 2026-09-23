# DotBot control application, rebuilt

Successor to `apps-sandbox/dotbot`, built up one layer at a time rather than
edited in place. At this stage it carries no control loop. It stays joined, polls the
position the secure side solves, samples the wheel encoders, advertises in the
same format as `apps-sandbox/dotbot`, and accepts direct motor commands. That is
all it does.

It exists for two reasons. It is an instrument for measuring the plant and the
position source before any control layer sits on top of them, and it is the base
those layers get added to, one at a time, each testable against only the layers
below it.

## What it does not do

No state estimator, no steering law, no waypoint sequencing, no heading state, no
displacement gate on incoming fixes. Those are added later, deliberately, and each
one has to earn its place against a measurement taken with this build.

## Structure

**One periodic tick.** `TICK_MS` (10 ms) drives a single RTC0 channel and every
slower activity divides it down: position at 100 ms, command timeout at 200 ms,
advertisement at 500 ms. The alternative, one channel per period, uses all three
usable RTC0 channels and leaves nothing for the encoder sampling rate a velocity
loop needs.

**The application is the position pipeline's clock.** `swarmit_keep_alive()` is
what runs the lighthouse solve and republishes it to shared data, so the rate that
call is made at *is* the position rate, and reading the position without having
just called it returns the previous solve. The two are deliberately adjacent in
`_position_poll()` and must stay that way. It also feeds the watchdog, so its
cadence is bounded above by the watchdog timeout as well.

**Freshness comes from a sequence, not from the coordinates.**
`swarmit_localization_get_fix()` returns the position together with the sequence
number of the solve it came from. The secure side advances that sequence by one
per published solve, so an unchanged sequence means the same measurement read
twice. The alternative, comparing coordinates, cannot tell a re-read from a robot
that has genuinely not moved, and the estimator this application grows into must
not run an update twice on one measurement. `swarmit_localization_get_position()`
still exists and still has its original signature; this application does not call
it.

The tick callback only increments a counter. The main loop compares it against
what it has serviced, drops any backlog rather than replaying it, and records the
worst backlog seen. A late tick is more useful reported than replayed, and on a
bench instrument that number is data.

**Advertisement is byte-identical** to the one `apps-sandbox/dotbot` emits, so
host-side parsing is unchanged. Fields this application does not own carry
unknown-value sentinels: heading is `-1000`, waypoints and waypoint index are
zero, control mode is manual. Encoder counts are totals since the previous
advertisement rather than since the previous control step, which is the same field
carrying the only meaning available here.

**Bench telemetry** is a second frame behind `DB_BENCH_TELEMETRY`, carrying the
RTC counter, the serviced tick number, the worst tick backlog, the raw solve, its
fix sequence, and the encoder totals. The sequence is what makes fix rate and fix
jitter measurable from the frames alone: its difference between two frames divided
by the tick difference is the rate. The frame is deliberately absent from
`protocol_data_type_t`: it must not exist in a shipped target, so it claims value
13 by local convention only. Do not register that value in the shared enum without
moving this first.

## Behaviour carried over deliberately

Rewriting rather than editing loses corrections nobody wrote down. These were
taken from `apps-sandbox/dotbot` and `apps/dotbot` on purpose, and the reasoning
is recorded here so the next rewrite does not have to rediscover them.

| Behaviour | Why it is here |
|---|---|
| `volatile` on anything an interrupt writes | The tick counter is written in the RTC callback and read in the main loop. Without it the compiler may cache the read and the loop never wakes. |
| `swarmit_keep_alive()` immediately before reading the position | It performs the solve and the republish, so it sets the position rate rather than merely keeping the application resident. Calling it less often than the position is read means reading the same solve twice. |
| Forwarding the SPIM4 interrupt to `swarmit_localization_handle_isr()` | That is how lighthouse sweeps are captured. A sandbox app that does not forward it gets no solves at all. |
| Coordinates above 100000 mm mean no solve | The secure side bounds-checks against the same figure and does not publish a solve outside it, so this is a second line rather than the only one. It stays because the bound is the only thing standing between a garbage solve and a position. |
| Encoder reads are destructive | `db_qdec_read_and_clear` empties the counter, so two consumers reading it steal counts from each other. Here there is one consumer and a running total; a second consumer needs a non-destructive sampling layer, not a second call. |
| RGB LED and QDEC behind board `#ifdef`s | Not every board in the family carries them. The flags are hardware presence, not features. |

## Behaviour deliberately changed

| Change | Reason |
|---|---|
| The command timeout is unconditional | In `apps-sandbox/dotbot` it is skipped in automatic mode, so an autonomously driving robot has no deadman at all. This application has no autonomous mode, so silence always means stop, and the exemption should not be reintroduced without a replacement. |
| Timeout arithmetic uses a masked difference | `db_timer_ticks()` returns a 24-bit counter that wraps every 512 s. A plain `now > then + delay` comparison is false for the entire pass after a wrap, so a robot whose last command arrived just before the rollover keeps its last commanded speed. |
| No displacement gate on incoming fixes | The gate in `apps-sandbox/dotbot` is anchored on the last accepted fix and only an accepted fix moves the anchor, so once the anchor is stale by more than the threshold, every fix that could correct it is rejected. Rejecting outliers belongs where the uncertainty is tracked, not against a self-referential anchor. |
| Freshness read from a sequence, not from the coordinates | `apps-sandbox/dotbot` can only compare coordinate values, which reads a stationary robot as having no new fix and a re-read of one solve as a measurement in its own right. |
| The device id is not read | It was retrieved at startup and never used. |

## Build

```
SEGGER_DIR="<install root>" BUILD_TARGET=sandbox-dotbot-v3 BUILD_CONFIG=Debug make dotbot-next
```

It links against `apps-sandbox/cmse_implib.a`, the secure gateway import library
the swarmit bootloader build produces. That blob and the bootloader on the robot
are one unit: the veneer addresses it names move whenever the set of gateway
functions changes, so a bot flashed with a bootloader older than the blob will
call the wrong entry point. Updating the blob means a cabled reflash of the
bootloader, and a rebuild of every application in `apps-sandbox/`, not only this
one.

Add `DB_BENCH_TELEMETRY` to the project's preprocessor definitions to enable the
second frame. Leave it off for anything that is not a bench run.
