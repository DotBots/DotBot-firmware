# DotBot-firmware

## Purpose

Firmware applications for DotBot, SailBot, FreeBot, XGo, lh2-mini-mote micro-robots, and nRF gateway DKs. Pure consumer of `DotBot-libs` (via git submodule) — pulls in drivers/BSP and produces per-target ELF/HEX artifacts. The recent commit history is dominated by "bump dotbot-libs to latest", which is a strong signal that this repo and `DotBot-libs` are conceptually one thing.

## Tech stack

- **Languages**: C (firmware), Python + Sphinx for docs
- **Targets**: Nordic nRF52833 / nRF52840 / nRF5340
- **Build**: SEGGER Embedded Studio (`emBuild`) from a top-level `Makefile`; Docker wrapper (`aabadie/dotbot:latest`) for reproducible CI builds. Per-board `.emProject` files at the repo root.
- **Style**: `clang-format`

## Submodules

This repo has **one** git submodule. After cloning, init it:

```bash
git clone --recurse-submodules https://github.com/DotBots/DotBot-firmware
# or, if already cloned:
git submodule update --init --recursive
```

| Submodule | Path | Pinned (snapshot 2026-05-12) |
|---|---|---|
| `DotBot-libs` | `dotbot-libs/` | `0.1.0-46-g1ebdd59` |

Recent commit history is dominated by "bump dotbot-libs to latest", so this submodule is intentionally kept current with `DotBot-libs/main`. `dotbot-lh2-calibration` pins the same SHA; `swarmit` lags 41 commits behind.

## Entry points

- `README.md` — submodule clone instructions, SEGGER setup
- `Makefile` — per-target project filtering, docker entry, format/doc targets
- `projects/dotbot/main.c` — flagship app showing how BSP/libs from `DotBot-libs` are wired together

## Build / run / test

```bash
git clone --recurse-submodules https://github.com/DotBots/DotBot-firmware
make BUILD_TARGET=dotbot-v2 BUILD_CONFIG=Release          # host SES
BUILD_TARGET=dotbot-v2 BUILD_CONFIG=Release make docker   # CI path
make artifacts                                             # collect ELF/HEX
make format / make check-format
make doc                                                   # Sphinx doc/sphinx/
```

CI: `.github/workflows/build.yml` — matrix over 12 targets × {Debug, Release}, plus `style`, `doc`, tag-triggered `release`. **No tests** (CI is build-only).

## Cross-repo dependencies

- **`DotBot-libs`** — git submodule at `dotbot-libs/`. Single hard dependency. `main.c` headers (`lh2.h`, `protocol.h`, `motors.h`, `tdma_client.h`, `control_loop.h`, etc.) all resolve into it.
- **`PyDotBot`** — referenced only as a runtime companion in READMEs and `doc/sphinx/conf.py` (no code dep)
- **`swarmit`** — only on the dead `adapt_to_swarmit` branch; not on `main`
- No references to: `mari`, `marilib`, `PyDotBot-utils`, `dotbot-lh2-calibration`, `dotbot-provision`, `qrkey`

## State of repo (snapshot 2026-05-05)

- Last commit on `main`: 2026-04-29
- Total commits on `main`: 1576 (large repo)
- Commits in last 90 days: 22 (last 365 days: 126). Actively maintained, primarily by `aabadie`.
- Branches (4 stale remote):
  - `171-integrate-ekf-into-the-dotbot-firmware` — 2022/2023 era, 1486 behind. Dead.
  - `32-implement-ultrasound-ranging-01bsp_us_ranging` — 2022 era. Dead.
  - `34-Implement-the-app-for-Ultrasound-ranging-...` — 2022 era. Dead.
  - `adapt_to_swarmit` — last 2025-03-21, 130 behind. Possibly salvageable if swarmit integration moves forward.
- TODO/FIXME/XXX/HACK: 0 in C/H/PY sources

## Hot spots and known gaps

- **Tight coupling to `DotBot-libs`**: this repo is essentially "apps for DotBot-libs". The two are strong consolidation candidates (see top-level AGENTS.md).
- **SES license-gated build**: Docker image build commented out in CI. Biggest onboarding friction post-summer-2026.
- **No tests at all** — CI only verifies it compiles.
- **4 stale remote branches**, 3 from 2022 should be pruned. `adapt_to_swarmit` worth investigating.

## Branch policy

- Default: `main`
- Three 2022-era branches (`171-...`, `32-...`, `34-...`) are dead. Safe to delete.
- `adapt_to_swarmit` — investigate before deleting; may carry useful integration work.

## Agent-task ideas

- **Host-side unit tests** for protocol/state-machine code that's separable from HAL.
- **SES → CMake/GCC migration** for at least one target. Highest-value onboarding unblock.
- **Delete dead 2022-era branches** (`171-...`, `32-...`, `34-...`).
- **Investigate `adapt_to_swarmit`**: rebase or delete.
- **Reconcile `Makefile` project list** with `DotBot-libs/Makefile` (the `03app_*` projects in `DotBot-libs/Makefile` actually live here).
- **Document board variants** — what's a DotBot-v2 vs v3 vs SailBot vs XGo at the firmware level (board init differences, motor wiring).

## Don't

- **Don't break the SES build** until a CMake/GCC equivalent is validated for at least one target.
- **Don't bump `DotBot-libs` submodule** without verifying it builds for *all* matrix targets — main.c uses headers from many drivers.
- **Don't merge `adapt_to_swarmit`** without coordinating with `swarmit` (the integration touches both).
