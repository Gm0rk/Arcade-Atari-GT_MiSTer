# Atari GT for MiSTer FPGA

A work in progress FPGA implementation of Atari Games' GT arcade hardware — the board under
**Primal Rage** and **T-MEK** — written for the
[MiSTer](https://github.com/MiSTer-devel) platform.

> **This core was made with AI.** The RTL, testbenches, reference models and
> documentation were written in collaboration with an AI assistant (Claude,
> by Anthropic), working from MAME's source and from measurements taken on
> the board. Every change was verified in simulation against a reference
> model and then on a DE10-Nano before being accepted.
> It is disclosed here so you can make your choice to use this core accordingly.

---

## Games

| Game | Versions (`.mra`) | Status |
|---|---|---|
| **Primal Rage** (1994) | 2.3 (Jan 1995), 2.3 (Dec 1994), 2.0 | **Playable, not finished.** Boots, runs attract, reaches character select and plays full fights with all seven fighters, with the real protection. Sound plays but runs slow on the heaviest screens, and the busiest scenes drop sprite lines. Tested on hardware with 2.3 (Jan 1995); the older two have their own control-panel layout and have not been tested on hardware yet |
| **T-MEK** (1994) | 5.1 (The Warlords), 5.1 prototype, 4.5, 4.4, 2.0 prototype | **Not playable.** 5.1 reaches its title and story screens, then stops; the 2.0 prototype cycles attract. Its protection device is unidentified even in MAME and is not implemented, and its sound data is not loaded yet |

---

## The core

| | |
|---|---|
| Main CPU | Motorola 68EC020 @ 25 MHz (board), with a small instruction cache as on the real chip |
| Sound | CAGE board: TMS320C31 DSP, 4 MB of sound data in SDRAM |
| Video | 336×240 visible in a 456×262 raster: **15.70 kHz / 59.92 Hz**, native, no scaler in the core |
| Layers | scrolling playfield tilemap, fixed alpha (text) tilemap, RLE-compressed scaled motion objects |
| Protection | Primal Rage's 136094-0004A, as an LFSR cipher model — see [Protection](#protection) |
| Settings | 28C16 EEPROM (2 KB), saved to the SD card |
| Controls | Quick High, Fierce High, Quick Low, Fierce Low, Start, Coin |
| OSD | aspect ratio, service mode, reset; the debug build adds a Debug page |

**Overall progress.** Percentages are honest estimates of remaining
effort, not lines of code; this block is kept in step with the
project's progress notes.

```
Scaffold / build       ████████████████████  100%
ROM loading (.mra)     ███████████████████░   98%
CPU  (68EC020)         ███████████████████░   97%
Memory subsystem       ████████████████████  100%
Video  (playfield)     ████████████████████  100%
Video  (alpha/text)    ████████████████████  100%
Video  (sprites)       ███████████████████░   97%
Sound  (CAGE C31)      ████████████████░░░░   80%
I/O and controls       ████████████████░░░░   80%
                       ────────────────────
Project                ████████████████░░░░   82%
```

---

## Using it

- **A 64 MB SDRAM module or larger is required.** The RLE sprite data alone
  is 32 MB; on a smaller module the core boots to a black screen.
- **ROMs are not included.** Use the MAME romsets with an `.mra` from
  `releases/` (Primal Rage) or `mra/` (all eight sets).
- **Two builds.** The release (`Arcade-Atari-GT`) has no debug features. The
  debug build (`Arcade-Atari-GT-Debug`) adds a Debug page to the OSD and an
  overlay, on by default: a build stamp and twelve hardware counters in the
  top-left. OSD → Debug → Debug Text turns it off. Its `.mra` files are in
  `mra/_Atari GT Debug/` and share the release's saved settings.
- **Native 15 kHz output.** The signal suits an arcade monitor or 15 kHz CRT
  but has not been tested on one yet. The scandoubler is not used, so a plain
  VGA monitor will most likely not sync.

## Known issues

- **The game runs slow on busy scenes.** The CPU reaches about 57% of a real
  25 MHz 68EC020 on a busy scene and 96% on a near-empty one; the remaining
  gap is competition for memory, not the processor.
- **The sound runs slow on the heaviest screens:** 85% of the real chip's
  pace on character select and 93% in a fight, so there it plays a little
  slow and low. The boot title keeps full pace and the attract reaches 92%;
  lighter screens reach full speed, and a governor stops them running past
  it.
- **The busiest scenes drop sprite lines** (the cave stage mid-fight): the
  sprite renderer runs past the frame, which shows as flicker and missing
  rows.
- **A vertical seam** down one column of the playfield on some scenes.
- **T-MEK does not play** (see [Games](#games)).

## Protection

Primal Rage's board carries a custom device, the **136094-0004A XGA**, in the
colour-RAM window at `0xD80000`; the game will not run without the right
answers. This core implements the model Andrea Bogazzi recovered for MAME in
September 2026 (credited below): a 16-bit LFSR cipher, verified against
7,649 golden vectors and confirmed on hardware with all seven fighters. It is
a fitted model rather than a solved device: the gaps in MAME's reference are
carried across rather than filled in. T-MEK's protection is a different,
unidentified device and is not implemented.

## Building

Quartus Prime 17.0.x (the MiSTer standard). Open `Arcade-Atari-GT.qpf` and
compile; when the compile meets timing, the core is moved to
`releases/Arcade-Atari-GT_YYYYMMDD.rbf`, dated by the build
(`post_flow.tcl`). The debug build is
`Arcade-Atari-GT-Debug.qpf`, the same source with one macro defined, and
makes `output_files/Arcade-Atari-GT-Debug.rbf`. The memory-init images the
build reads are in `mem/`.

```
Arcade-Atari-GT.sv    core top level: OSD, debug overlay, boot sequencing
rtl/cpu/              68EC020
rtl/board/            memory map, protection, SDRAM controller, ROM loader, EEPROM
rtl/video/            raster timing, colour RAM, playfield, alpha, RLE objects, mixer
rtl/cage/             the CAGE sound board: TMS320C31 core, caches, mailboxes, DAC
sys/                  the MiSTer framework (unmodified)
mra/, releases/       MRA files; releases/ also holds the core
mra/_Atari GT Debug/  the same MRA files for the debug build
```

---

## Credits and references

This core is a reimplementation. It would not have been possible without:

**[MAME](https://www.mamedev.org/)** — the reference for essentially all
hardware behaviour. Specifically:

| File | Author | Used for |
|---|---|---|
| `atarigt.cpp`, `atarigt.h` | Aaron Giles | memory map, machine configuration, protection, MO command register |
| `atarigt_v.cpp` | Aaron Giles | video registers, per-scanline scroll, the Primal Rage colour mixer |
| `atarirle.cpp`, `atarirle.h` | Aaron Giles | the RLE object engine — list scan, object table, decode, scaling, flip |
| `eeprom.cpp`, `eeprompar.cpp` | Aaron Giles | 28C16 behaviour: erased-state default, lock-after-write |
| `cage.cpp` | Aaron Giles | CAGE communication registers |
| `tms320c3x.cpp`, `320c3x_ops.ipp` | Aaron Giles | the CAGE DSP (TMS320C31) |
| `atarixga.cpp`, `atarixga.h` | Andrea Bogazzi | **the 136094-0004A protection device** — the LFSR structure, clock-count table, key permutation and per-character taps, fitted to 4,177 plaintext/ciphertext pairs recovered from the PlayStation port |
| `m68020.cpp` (Musashi) | Karl Stenerud | the CPU that produced the execution trace the 68020 model was validated against |

MAME is a reference for *behaviour*; no MAME code is compiled into this core.

**Primal Rage's protection was substantially worked out by Andrea Bogazzi** and
merged into MAME in September 2026 —
[pull request #16105](https://github.com/mamedev/mame/pull/16105) and
[commit `8c1345d`](https://github.com/mamedev/mame/commit/8c1345d8f5af207fd3fae62a928782d46f2d126b)
— replacing the behavioural model this core previously reproduced. It is a much better model
rather than a solved device — the feedback taps are a five-entry lookup whose
derivation is unknown, and 57% of the key-byte table was never exercised by the
game and is blank. The real device is a 16-bit
Fibonacci LFSR clocked 16–125 times, the count from a 128-entry table, the
feedback taps selected by a word the game writes before each query, and the key
byte drawn from 2 KB of uploaded key material by a bit permutation of the query
index. It was recovered by matching the game's own ciphertext tables against
the plaintext the PlayStation port stores in their place — 4,177 pairs, all
reproduced but three. The same commit also corrected the sprite palette index
to align the pen base to each object's colour depth, which fixed Primal Rage's
blood colour; **this core now applies the same mask, confirmed on hardware —
blood renders red.** See [Protection](#protection).

**[MiSTer](https://github.com/MiSTer-devel/Main_MiSTer)** — the framework, and
`Template_MiSTer` by Alexey Melnikov (**Sorgelig**), whose `sys/` directory
provides the HPS interface, video scaler, and SDRAM pin handling this core
builds on. `sys/` is unmodified. `Main_MiSTer`'s own source (`user_io.cpp`,
`menu.cpp`, `support/arcade/mra_loader.cpp`) was read to establish how the
framework sequences reset and NVRAM restore at core load — behaviour this
core has to match and that is not otherwise documented.

The MiSTer community's existing arcade cores were a useful model for project
structure and MRA conventions.

Board details and the full game list are catalogued at
[System 16 — Atari GT hardware](https://www.system16.com/hardware.php?id=777).

## License

The core RTL is released under the GNU General Public License v2.0 or later,
consistent with the MiSTer framework it builds on. See `LICENSE`.

`sys/` retains its original license and authorship.

No ROM data is included or distributed.
