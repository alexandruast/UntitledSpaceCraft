# Handoff: this fork and its controller

Start here in a new session. Keep this file current when the state changes. It is fork-only (upstream has no
HANDOFF.md), so upstream merges won't touch it.

## What this repo is

- The user's fork of CodapopKSP/UntitledSpaceCraft (USC), a modular KSP controller.
  - `origin` = github.com/alexandruast/UntitledSpaceCraft (push here).
  - `upstream` = github.com/CodapopKSP/UntitledSpaceCraft.
- All work on the user's own controller lives in this repo, not in the notes repo.
- The user's controller has 8 modules: Action Groups, Telemetry, Control Systems, Time, Rotation, Translation,
  Throttle, Executive Actions. The hub is an Arduino Micro.

## State (2026-11-13)

- Synced with upstream at Hub v3.1 (merge `c64309c`). Local branch `backup/pre-upstream-sync-2026-11-13` holds the
  pre-merge main.
- Custom telemetry is done and committed (`7f3b34e`). Both boards were flashed from it on 2026-11-13.
- **Not yet tested in KSP.** Checklist for the first flight:
  - screen blank outside flight;
  - AL/VL page shows AoA `---` on the pad;
  - MNVR / ORBT / FUEL switch within about 0.1 s, with no leftover values from the previous page;
  - SOI name (e.g. "Kerbin") on ORBT;
  - screen blanks back at the space center and comes back by itself on the next flight.

## Layout

| Path | What |
|---|---|
| `Firmware/Container Firmware/Hub_3_1/` | Hub source. The base files, plus module snippets in `Modules/` |
| `Firmware/Container Firmware/Hub_3_1/Modules/c1.ino` | Telemetry (LCD) snippet: our 4-page handlers |
| `Firmware/Container Firmware/assemble_hub.py` | Pastes the snippets into a flashable sketch (stand-in for the untitledspacecraft.com code tool, which only knows upstream code) |
| `Firmware/Container Firmware/My_Controller/Hub_3_1/` | GENERATED hub sketch for this controller. Never edit; regenerate |
| `Firmware/Container Firmware/My_Controller/README.md` | Module map, plus the user-facing key for the telemetry pages and labels |
| `Firmware/Module Firmware/Telemetry/Telemetry.ino` | Telemetry display firmware (U8g2, ST7920 128x64) |

## Telemetry design (hub + display must stay paired)

- **Protocol:** 20 fields of at most 8 chars, sent as 5 I2C packets of 4 fields: `[packet#][len][chars]...`.
  After packet 4 the hub reads 1 byte back, which is the page the display is showing.
- **Field map:** `LCD_F_*` in `Hub_3_1/definitions.h` and `F_*` in `Telemetry.ino` must match. Fields 1-7 (APO,
  PER, TTA, TTP, dVT, dVS, ALT) mean the same on every page. Fields 8-19 are page-specific.
- **Field 0 = page tag:** `'1'..'4'` for the page the data belongs to, or `"X"` when not flying (display blanks).
  - The display shows values only when the tag matches its page.
  - The hub resets and re-tags all fields on a page change (`LCD_resetFields`).
  - The hub restores the tag after a blank (`LCD_syncTag`, called from the c1 transmission line).
- **Subscriptions:** the hub subscribes to Simpit channels per page (`registerTelemetryChannelsForMode`).
- **SOI name:** cached in `LCD_soi`, because KSP only sends it on change.
- **Pages / buttons:** 1 AL/VL (flight), 2 MNVR, 3 ORBT, 4 FUEL. The display's mode numbers equal the page
  numbers (stock firmware used 0 for FUEL; not anymore).
- **Formatting is on the hub** (`02_Transmissions.ino`):
  - `lcd_units`: whole number with a k/M/G/T suffix past 5 digits;
  - `lcd_f`: decimals, falling back to `lcd_units` when the value won't fit;
  - `lcd_time_until`: mm:ss, hh:mm, then d / y; `---` if negative;
  - `lcd_resource`: percent, or `-` when none aboard.
- **AoA:** computed on the display as the spherical angle between the nose and the surface velocity.
  `---` below 1 m/s.

## Build and flash

Toolchain: Arduino IDE installed. Use the CLI bundled with it:
`~/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe`.
Libraries: KerbalSimpit 2.4.0, Keyboard 1.0.7, U8g2 2.36.19 (all in `~/Documents/Arduino/libraries`).

```sh
python "Firmware/Container Firmware/assemble_hub.py" --compile   # regenerate My_Controller/Hub_3_1 + compile (Micro)
CLI=".../arduino-cli.exe"
"$CLI" board list                                                # COM ports change after resets/replugs
"$CLI" upload -b arduino:avr:micro -p COM8 "Firmware/Container Firmware/My_Controller/Hub_3_1"
"$CLI" compile -b arduino:avr:nano:cpu=atmega328old -u -p COM6 "Firmware/Module Firmware/Telemetry"
```

- **Hub:** its own USB port. It was COM5, and became COM8 after a reset.
- **Telemetry module:** flashed through an FTDI adapter (VID 0403 / PID 6001), which shows up as COM6.
  - Its COM port can take a moment to appear after plugging in.
  - Only `cpu=atmega328old` (57600 baud) syncs.
- **Sizes at `7f3b34e`:**

| Board | Flash | RAM |
|---|---|---|
| Hub | 25,534 / 28,672 B (89%, about 3.1 KB left) | 1,206 B |
| Display | 18,218 / 30,720 B | 1,126 / 2,048 B |

  Hub flash is the limit for future features.
- **No desktop C++ compiler** is installed. Only the AVR builds compile; nothing runs on the PC.
- **External PSU:** regulated 5 V, 2.1 mm barrel jack, centre positive. The jack feeds the modules' 5 V line
  directly (no regulator), so never use 9 V or 12 V. The docs give no current; 2 A is ample (estimate).

## Syncing with upstream

```sh
git fetch upstream && git merge upstream/main
```

- Expect conflicts in Hub_3_1 `definitions.h`, `01_System_Functions.ino`, `02_Transmissions.ino`,
  `Modules/c1.ino`, `settings.h` and `Hub_3_1.ino`.
- After merging, rerun `assemble_hub.py --compile` and commit the regenerated `My_Controller/Hub_3_1`.

Local differences from upstream to keep:
- the telemetry system above;
- the user's throttle calibration in `settings.h` (338 / 864 / 306 / 774);
- the `Hub_3_1.ino` EVA release loop, which uses the size of `EVA_List_Analog`. Upstream loops to 11, but the
  Translation snippet defines 10.

## Assembler quirks (learned from upstream snippets)

- Snippet sections: `// Transmissions`, `// Enable Action`, `// Action`, `// Message Handler`,
  `// Keyboard Emulation`, `// definitions`, `// Action Results`.
- `modules.h` must order them: Keyboard Emulation, then definitions, then Action Results.
- The action-group handlers (h1/h2/h3/a4, which use `myAG`) go inside one shared `CAGSTATUS_MESSAGE` case.
- A Translation snippet (f5) that defines `EVA_List_Analog` replaces the default one in `actionResults.h`.
- "all" modules does not compile: upstream snippets have duplicate cases and alternative modules. Only build real
  module sets.

## Gotchas

- Arduino adds function declarations before typedefs. Don't use a typedef in a function signature; write the full
  type.
- Some Hub_3_1 files have mixed CRLF/LF line endings (git normalizes them). Match `\r` when scripting edits.
- Deliberately not ported from the old Hub 2.3 WIP:
  - removing the second write to the Ctrl Sys display in `module_transmission` (upstream kept it in 3.1);
  - the `int` to `uint8_t` cleanups in the module lists.

## Ideas not done

- Stage-only fuel (`LF_STAGE_MESSAGE` etc.) on the FUEL page.
- Numeric percentages on the FUEL page.
- A landing view (surface altitude, which is in `altitudeMessage.surface` but not used yet).
- Burn countdown alerts.
- Check hub flash before adding anything.
