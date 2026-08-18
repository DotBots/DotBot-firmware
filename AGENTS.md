# DotBot-firmware - AI agent guide

Firmware for the DotBot micro-robots (and siblings: SailBot, FreeBot, XGO, the
LH2 mini-mote) plus the nRF gateway/DK apps. C, bare-metal on Nordic nRF52 /
nRF5340, built with SEGGER Embedded Studio. This file orients an agent working
in this repo; broader cross-repo conventions (build discipline, commit style,
fork/PR flow) come from the agentic workspace this repo is checked out into.

## What's here

- **`apps/`** - the **bare** applications (talk directly to the radio + chip, no
  sandbox). Key ones: `apps/dotbot` (the standard DotBot robot app), `dotbot_gateway`
  / `dotbot_gateway_lr` (nRF DK as a radio gateway), `sailbot`, `freebot`, `xgo`,
  `lh2_calibration` (the LH2 calibration firmware), `lh2_mini_mote_*`, `nrf5340_net`
  (network-core image), `log_dump`.
- **`apps-sandbox/`** - the same robot apps built as **TrustZone non-secure
  images** that run *inside* the SwarmIT sandbox (`dotbot`, `dotbot-simple`,
  `move`, `motors`, `rgbled`, `spin`). They link against `cmse_implib.a` (the
  Non-Secure-Callable import lib produced by `swarmit`). Absorbed from the old
  `dotbot-swarmit` repo.
- **`dotbot-libs/`** - submodule (`DotBots/DotBot-libs`): BSP + drivers. The
  **control loop math lives here** (`dotbot-libs/drv/control_loop/`), shared by
  bare and sandbox builds. `swarmit` pins the *same* `dotbot-libs`, so the
  `control_loop.c` you read here is byte-identical to the copy under
  `swarmit/dotbot-libs/`.
- **`*.emProject`** - one SES solution per target/board: `dotbot-v1/v2/v3`,
  `sandbox-dotbot-v2/v3`, `sandbox-nrf5340dk`, `nrf5340dk-app/net`,
  `nrf52833dk`, `nrf52840dk`, `freebot-v1.0`, `sailbot-v1`, `xgo-v1/v2`,
  `lh2-mini-mote`.

## The control loop (read this before touching motion/LH2/waypoint code)

This is the single most load-bearing, least-obvious thing in the repo, and it
spans three repos. It is a **two-tier layered control loop**: a fast loop on the
bot, a slow loop on the Python host. Getting the tiers and rates wrong is the
root of most "why doesn't the robot go where I told it" confusion.

### Who closes which loop

- **Fast inner loop - ON THE BOT (this repo).** The bot computes its own LH2
  `(x, y)` position locally and runs a heading controller toward *the last
  waypoint it was given*, **autonomously** - it keeps driving to the goal even
  while it hears nothing new from the host. The host is **not** in this loop.
- **Slow outer loop - in Python (PyDotBot).** The host reads each bot's position
  (by REST-polling the controller) and sends **waypoints / goals**, not per-step
  motor commands. It plans; the bot executes.

So `move_to(x,y)` / waypoint following is the *primary* control path; the host
sending a goal at ~1-5 Hz is enough because the bot self-navigates between
goals. Direct motor teleop (`move_raw`) is the exception, used only for joystick/
keyboard driving of a single bot.

### Rates (from firmware constants)

| Rate | Bare (`apps/dotbot/main.c`) | Sandbox (`apps-sandbox/dotbot/main.c`) |
|---|---|---|
| Inner control step (`update_control` + motor write) | **~250 ms** - gated on the LH2 trigger, `5 * DB_LH2_UPDATE_DELAY_MS` (50) on RTC ch1 (`main.c:42,206,252`) | **~100 ms** - `DB_POSITION_UPDATE_DELAY_MS` (100) on RTC ch1 (`main.c:36,178,211`) |
| On-bot LH2 position refresh | ~250 ms (same trigger; computed app-side via `db_lh2_calculate_position`) | ~100 ms (read from the secure side via NSC `swarmit_localization_get_position`; the bot can't touch LH2 directly) |
| App advertisement (position/telemetry UP the radio) | **500 ms** - `DB_ADVERTIZEMENT_DELAY_MS` (RTC ch2) | **500 ms** - same constant |
| Manual-command deadman (stops motors if no packet) | ~520 ms (`17000` RTC ticks / 32768) | ~520 ms |
| swarmit netcore STATUS frame | n/a | **~1 s** - `mr_timer_hf_set_periodic_us(..., 1000000, _send_status)` in `swarmit/device/network_core/Source/main.c` |

The control step is **event-gated on a fresh valid LH2 fix and runs in AUTO mode
only** - so the real rate is "at most once per trigger," skipped on a missed
fix. Both bare and sandbox use the RTC (`dotbot-libs/bsp/nrf/timer.c`), not a
high-frequency timer, for these periods.

### The steering law

In `dotbot-libs/drv/control_loop/control_loop.c`, `update_control()`. It is
**proportional + derivative (PD) heading control** (no integral term):
`error_angle = angle_to_target - direction`, `angular_speed = (error/180)*P +
(d_error/180)*D`, fed differentially into `pwm_left/right`. Gains are
**board-specific** (`control_loop.c:10-31`; e.g. dotbot-v3: `DB_MAX_PWM=60`,
`P=1.0`, `D=0.3`). Heading comes from successive LH2 fixes (`compute_angle`), not
an IMU. Two compile-time variants change *only* the target point, not the rate
or the PD math:

- **`DOTBOT_CONTROL_LOOP_USE_PURE_PURSUIT`** - defined in the **sandbox**
  `.emProject`s. Aims at a lookahead point `1.5 * threshold` along the current
  segment instead of the raw next waypoint -> smoother curves.
- **`DOTBOT_CONTROL_LOOP_USE_EKF`** - a 3-state `[x,y,theta]` EKF fusing encoder
  odometry + LH2. **Not enabled in any firmware** - only in PyDotBot's host-side
  sim build (`PyDotBot/utils/control_loop`, default off). The sandbox app has no
  QDEC/encoders at all, so even if enabled its predict step would get zero
  odometry. Bare dotbot enables **neither** -> plain PD toward the raw waypoint.

### AUTO vs MANUAL mode (what the host polls)

Set in `radio_callback()` by packet type:

- `DB_PROTOCOL_CMD_MOVE_RAW` -> maps joystick `left_y`/`right_y` (±INT8_MAX) to
  ±100 PWM and drives motors directly (teleop).
- `DB_PROTOCOL_LH2_WAYPOINTS` with `count > 0` -> `ControlAuto`; the inner loop
  runs. Empty list -> stop + `ControlManual`.
- When the **last waypoint is reached** (`update_control` sets `all_done`), the
  app flips back to `ControlManual` and stops. **This AUTO->MANUAL flip is the
  "arrival" signal the Python examples poll for** to sequence the next waypoint
  batch (≤12 per `DB_PROTOCOL_LH2_WAYPOINTS` packet).

### The two-namespace position split (a real, load-bearing gotcha)

Position leaves the bot on **two different Mari `next_proto` namespaces**, read
by two different host clients:

- **DOTBOT_APP advertisement** (`next_proto = 0x11`, ~500 ms): the standard
  `DB_PROTOCOL_DOTBOT_ADVERTISEMENT` (direction + `(x,y)` + battery + ...).
  **PyDotBot's controller consumes this** (`dotbot/adapter.py` accepts only
  `0x11`). This is the dotbot controller's *only* position source.
- **swarmit netcore STATUS** (`next_proto = 0x10`, ~1 s): `SWRMT_MSG_STATUS`
  carrying device/status/battery + `position_2d_t {x,y}` (mm). **Only the
  swarmit client consumes this** (`swarmit/.../testbed/adapter.py` accepts only
  `0x10`); PyDotBot drops it.

=> The swarmit STATUS frame carries a perfectly good position that the **dotbot
controller never sees**, because the two adapters filter on disjoint `next_proto`
values. If you're debugging "the dotbot dashboard isn't showing position during
a swarmit run," this split is why. Note: **both frames carry the same position
number** (the sandbox app's `swarmit_localization_get_position()` reads the very
`ipc_shared_data.current_position` the STATUS frame reports), so reading `0x10`
would give no *fresher* fix - it's the slower frame. The real cost is **airtime**:
a RUNNING sandbox bot sends ~3 redundant uplink pkt/s (status 1 Hz + adv 2 Hz),
which is the constraint at 100+ bots. The direction under discussion is to
**fold the two into one packet** (the app stages an opaque telemetry blob that
the secure side appends to its STATUS frame), not to read both - so don't
"resolve" the split by adding a second ingest path without checking that work.

### Host side, for reference (PyDotBot)

The examples confirm the model: ORCA demos loop at ~5 Hz and feed each step as a
fresh **waypoint** (never `move_raw`); multi-step demos are batch-gated (send a
batch, watch AUTO->MANUAL for "done"). The protocol allows `DB_MAX_WAYPOINTS`
(16) per packet; the examples send fewer, since a full packet is ~149 bytes.

Robot state leaves the controller **two ways, both in use**:

- **Pushed** over the `/controller/ws/status` WebSocket. Every advertisement
  carrying a new position raises an `UPDATE` notification whose payload is the
  full `DotBotModel`, including `lh2_position` and `mode`. This is what keeps
  the dashboard live.
- **Polled** via `GET /controller/dotbots`. This is what the Python examples
  use when they batch waypoints and wait for "done".

A second WebSocket, `/controller/ws/dotbots`, runs the other way: it is command
**ingress**, accepting RGB LED, `move_raw` and waypoint messages.

Note what none of that changes: **the robot never signals arrival.** It
advertises state every 500 ms, and the host infers "done" from the `mode` field
having flipped AUTO->MANUAL. The arrival event is synthesised host-side; nothing
on the wire announces it.

Teleop (`dotbot/keyboard.py`, `joystick.py`) is pure `move_raw` at ~20 Hz.

### Key files

- `apps/dotbot/main.c` - bare app: timers, `radio_callback`, `_update_control_loop`.
- `apps-sandbox/dotbot/main.c` - sandbox app: NSC position read, `swarmit_keep_alive`.
- `dotbot-libs/drv/control_loop/control_loop.c` + `control_loop.h` - the PD /
  pure-pursuit / EKF math (shared, byte-identical across checkouts).
- `swarmit/device/network_core/Source/main.c` - the `_send_status` (0x10) path.

## Build / run / test

Firmware is built with **SEGGER Embedded Studio** (`emBuild`) wrapped by the
Makefile. Per the workspace convention, build **locally with SES, never `make
docker`** on Apple Silicon (the image is `linux/amd64` under QEMU; `make docker`
is the CI path only):

```bash
SEGGER_DIR="/Applications/SEGGER/SEGGER Embedded Studio 8.22a" \
  BUILD_TARGET=dotbot-v3 BUILD_CONFIG=Release make
make list-targets        # valid BUILD_TARGETs (derived from *.emProject)
make help                # the knobs (BUILD_TARGET, BUILD_CONFIG, BUILD_MODE)
make artifacts           # collect built .hex/.bin into artifacts/
make check-format        # clang-format -style=file
```

`BUILD_MODE` toggles incremental vs `-rebuild` (added in #412). There are **no
host-side unit tests** for the C - CI only verifies it compiles across the
target matrix; logic is not exercised. Flashing is via SES / J-Link, or via
`dotbot device flash` in PyDotBot (chip-family/core handled by its `BoardSpec`
table - note e.g. **dotbot-v2 is an nRF5340**, not nRF52833; the truth is each
`.emProject`'s `arm_target_device_name`).

## Cross-repo coupling

- **`dotbot-libs`** submodule - shared BSP + the control loop. Don't drift the
  pin without checking `swarmit`'s pin (they must agree for `control_loop.c` to
  stay identical).
- **`swarmit`** produces `cmse_implib.a`, which `apps-sandbox/*` link against;
  sandbox apps call NSC entries (`swarmit_keep_alive`, `swarmit_localization_*`,
  `swarmit_send_raw_data`). A swarmit NSC-API change ripples here.
- **`mari`** - the radio link layer; sandbox traffic rides Mari frames with the
  `next_proto` namespaces above. **`PyDotBot`** consumes the `0x11`
  advertisement.

## Conventions

- **Commits**: `realm: imperative` where the realm is a path - existing history
  uses `apps/dotbot:`, `apps-sandbox/dotbot:`, `makefile:`, `sandbox:`. For this
  file, `agents:`. (See workspace `AGENTS.md` for the full rule + the
  `AI-assisted:` trailer.)
- **C style**: `clang-format -style=file` (`make format` / `check-format`).
- **ISR-set flags must be `volatile`** (recent fixes: `apps/dotbot`,
  `apps-sandbox/dotbot`).

## Don't

- Don't add work to the LH2/control ISRs or the timer callbacks - the inner loop
  is timing-sensitive; latency there desyncs the control step.
- Don't enable `DOTBOT_CONTROL_LOOP_USE_EKF` in a firmware build expecting it to
  work in the sandbox (no encoders there).
- Don't "fix" the two-namespace position split here unilaterally - it's a
  cross-repo (PyDotBot + swarmit adapter) decision; see the control-loop section.
- Don't run `make docker` locally (CI-only; slow under QEMU).
