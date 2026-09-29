# DotBot-firmware - AI agent guide

Firmware for the DotBot micro-robots (and siblings: SailBot, FreeBot, XGO, the
LH2 mini-mote) plus the nRF gateway/DK apps. C, bare-metal on Nordic nRF52 /
nRF5340, built with SEGGER Embedded Studio. This file orients an agent working
in this repo; broader cross-repo conventions (build discipline, commit style,
fork/PR flow) come from the agentic workspace this repo is checked out into.

## What's here

- **`apps/`** - the **bare** applications (talk directly to the radio + chip, no
  sandbox). Key ones: `apps/dotbot` (radio-driven wheel speeds, raw duty and
  the RGB LED; the teaching app), `dotbot_gateway`
  / `dotbot_gateway_lr` (nRF DK as a radio gateway), `sailbot`, `freebot`, `xgo`,
  `lh2_calibration` (the LH2 calibration firmware), `lh2_mini_mote_*`, `nrf5340_net`
  (network-core image), `log_dump`.
- **`apps-sandbox/`** - robot apps built as **TrustZone non-secure
  images** that run *inside* the SwarmIT sandbox (`dotbot`, `dotbot-simple`,
  `move`, `motors`, `rgbled`, `spin`). They link against `cmse_implib.a` (the
  Non-Secure-Callable import lib produced by `swarmit`). Absorbed from the old
  `dotbot-swarmit` repo.
- **`dotbot-libs/`** - submodule (`DotBots/DotBot-libs`): BSP + drivers. The
  **control math lives here**: `drv/dotbot_control/` (the sandbox app's control
  core) and the layers it ties together, `drv/wheel_control/` (which the bare
  app uses on its own), `drv/pose_estimator/` and `drv/steering/`.
- **`*.emProject`** - one SES solution per target/board: `dotbot-v1/v2/v3`,
  `sandbox-dotbot-v2/v3`, `sandbox-nrf5340dk`, `nrf5340dk-app/net`,
  `nrf52833dk`, `nrf52840dk`, `freebot-v1.0`, `sailbot-v1`, `xgo-v1/v2`,
  `lh2-mini-mote`.

## The control loop (read this before touching motion/LH2/waypoint code)

It spans three repos and is a **two-tier layered control loop**: a fast loop on
the bot, a slow loop on the Python host. Getting the tiers and rates wrong is
the root of most "why doesn't the robot go where I told it" confusion.

**Two apps.** Only the sandbox `apps-sandbox/dotbot` follows waypoints. It is a
thin hardware layer around DotBot-libs' `drv/dotbot_control`, a hardware-free
core running the per-wheel speed loop (`drv/wheel_control`), the pose estimator
on the encoders and LH2 (`drv/pose_estimator`) and the onboard steering
(`drv/steering`). PyDotBot's simulator runs the same core compiled to
WebAssembly. The app's `README.md` is the reference for its drive modes,
waypoint batches and max speed. The bare `apps/dotbot` has no localization and
no waypoints: it drives wheel speeds through `drv/wheel_control` or raw duty,
and sets the RGB LED (its `README.md`).

### Who closes which loop

- **Fast inner loop - ON THE BOT (this repo).** The sandbox bot reads its LH2
  fix from the secure side, estimates its pose and steers along the **batch of
  waypoints** it was given, **autonomously** - it keeps driving while it hears
  nothing new from the host, and stops on arrival or on its own timeouts. The
  host is **not** in this loop.
- **Slow outer loop - in Python (PyDotBot).** The host reads each bot's state
  from the controller and sends **waypoint batches / goals**, not per-step motor
  commands. It plans; the bot executes.

So waypoint following is the *primary* control path. Direct driving
(`move_raw` duty, `wheel_velocity` speeds) is the exception: joystick and
keyboard teleop, or a host closing its own loop, and it stops ~520 ms after the
last command.

### Rates (from firmware constants)

| Rate | Bare (`apps/dotbot/main.c`) | Sandbox (`apps-sandbox/dotbot` + `drv/dotbot_control`) |
|---|---|---|
| Wheel speed loop | **10 ms** (`TICK_MS`) | **10 ms** (`DB_CONTROL_TICK_MS`), with the estimator predict |
| Steering step | n/a | 100 ms |
| On-bot LH2 position refresh | n/a | ~100 ms (read from the secure side via NSC `swarmit_localization_get_fix`, with its fix sequence; the bot can't touch LH2 directly) |
| App advertisement (telemetry UP the radio) | **500 ms** (`ADVERTISEMENT_TICKS`) | **100-1000 ms**, from the node's minimum TX interval (500 ms while not joined) |
| Direct-drive deadman (stops motors if no command) | ~520 ms | ~520 ms |
| swarmit netcore STATUS frame | n/a | **~1 s** - `mr_timer_hf_set_periodic_us(..., 1000000, _send_status)` in `swarmit/device/network_core/Source/main.c` |

Both apps pace everything from one RTC tick (`dotbot-libs/bsp/nrf/timer.c`),
not a high-frequency timer.

### Waypoint batches and arrival (what the host polls)

`DB_PROTOCOL_LH2_WAYPOINTS` carries up to `DB_MAX_WAYPOINTS` (16) points plus an
optional trailer: a batch id, per-point headings (a point with a heading is a
pose), a heading tolerance and a pass radius. A resent batch whose id the bot
already has is ignored, so the host resends until the advertisement confirms
it. `DB_PROTOCOL_CMD_MAX_SPEED` sets the cruise speed of waypoint moves.

Arrival is explicit: the sandbox advertisement ends with a **waypoint report**
(`protocol_waypoints_report_t`) carrying the batch status (`IN_PROGRESS`,
`ARRIVED`, `FAILED` or `ABORTED` with a reason), the batch id, the max speed and
the estimated axle midpoint. The control mode also reads AUTO while a batch is
active. The bare app's advertisement has no report and always reads MANUAL.

### The two-namespace position split (a real, load-bearing gotcha)

Position leaves the bot on **two different Mari `next_proto` namespaces**, read
by two different host clients:

- **DOTBOT_APP advertisement** (`next_proto = 0x11`, 100-1000 ms): the standard
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
a swarmit run," this split is why. Note: **both frames come from the same LH2
solve** (the advertisement carries the estimator's position while it tracks,
else the last solve read with `swarmit_localization_get_fix()`), so reading
`0x10` would give no *fresher* fix - it's the slower frame. The real cost is
**airtime**: a RUNNING sandbox bot sends a STATUS frame per second on top of its
1-10 advertisements per second, which is the constraint at 100+ bots. The direction under discussion is to
**fold the two into one packet** (the app stages an opaque telemetry blob that
the secure side appends to its STATUS frame), not to read both - so don't
"resolve" the split by adding a second ingest path without checking that work.

### Host side, for reference (PyDotBot)

Teleop (`dotbot/keyboard.py`, `joystick.py`, the console joystick) is pure
`move_raw` at ~20 Hz. The controller's waypoint routes send batches and read
completion from the waypoint report.

For how the controller exposes robot state, see "Controller surface" in
PyDotBot's `AGENTS.md`.

### Key files

- `apps/dotbot/main.c` + `README.md` - bare app: the tick, the radio mailbox,
  the speed loop and the advertisement.
- `apps-sandbox/dotbot/main.c` + `README.md` - sandbox app: the tick scheduler,
  NSC fix read, `swarmit_keep_alive`, the mailbox into the control core.
- `dotbot-libs/drv/dotbot_control/` - the control core: drive modes, deadman,
  batch dedup, the advertisement and waypoint report.
- `dotbot-libs/drv/wheel_control/`, `drv/pose_estimator/`, `drv/steering/` - the
  core's layers, each with a host test (`make test` in DotBot-libs).
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

- **`dotbot-libs`** submodule - shared BSP + the control core. `swarmit` pins
  it too; check its pin when moving this one.
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

- Don't add work to the ISRs or the timer callbacks - both apps keep them to a
  counter or a mailbox copy and run the control in the main loop.
- Don't "fix" the two-namespace position split here unilaterally - it's a
  cross-repo (PyDotBot + swarmit adapter) decision; see the control-loop section.
- Don't run `make docker` locally (CI-only; slow under QEMU).
