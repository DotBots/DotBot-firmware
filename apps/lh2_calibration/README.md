# LH2 calibration firmware

Streams raw Lighthouse 2 sweep data over UART as HDLC frames. Companion
firmware for the `dotbot calibrate-lh2` TUI (from the
[PyDotBot](https://github.com/DotBots/PyDotBot) package), which uses
the streamed counts to compute per-basestation homography matrices
saved to `~/.dotbot/calibration/`.

Folded in from the standalone `dotbot-lh2-calibration` repo.

## Usage

1. Flash this firmware on a DotBot (or any LH2-equipped board) that has
   visibility on the lighthouses you want to calibrate.
2. Connect the DotBot via USB-serial (default UART baudrate `115200`).
3. Run `dotbot calibrate-lh2` from a host with `pip install
   dotbot[calibrate]`. Follow the TUI prompts to capture the four
   reference points per basestation.

## Wire format

Each frame is HDLC-encoded and contains:

- 1 byte: basestation index (`lh_index`, 0-based)
- N × 4 bytes: `lfsr_counts` for each sweep (`uint32_t` little-endian),
  where N is `LH2_SWEEP_COUNT` (defined in `dotbot-libs/bsp/lh2.h`)

Frames are emitted only when both sweeps of a basestation have
fresh `DB_LH2_PROCESSED_DATA_AVAILABLE` data; missing-data frames are
suppressed.
