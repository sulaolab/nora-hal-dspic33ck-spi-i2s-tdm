# nora-hal-dspic33ck-spi-i2s-tdm — SPI framed-mode I2S/TDM transport HAL

**NORA-HAL** — *Native On-chip Resource Assistant*

A compact, reusable SPI/I2S/TDM **transport** HAL for dsPIC33CK — part of **NORA-HAL**,
a HAL family whose public API is namespaced `nora_*` / `NORA_*`. It moves audio frames
over a framed SPI peripheral with DMA ping-pong and a per-instance block callback. It
is intentionally **small**: it does not try to be a turnkey "drop-in and forget" audio
stack. Board-specific, failsafe, and CMSIS-SAI buffer-semantics concerns stay in layers
above it, so a project can extend only what it needs.

## 0. Naming

The public API is `nora_*` / `NORA_*`.

The chip name survives in exactly two places, both deliberate:

- **Implementation file names** carry a backend tag: `nora_spi_i2s_tdm_dspic33ck.c` is
  the dsPIC33CK backend of the processor-neutral `nora_spi_i2s_tdm.h`. A second
  processor would add `nora_spi_i2s_tdm_<tag>.c` beside it, not a second header. The
  tagged `_dspic33ck_hw.{c,h}`, `_dspic33ck_reg.h`, `_dspic33ck_reg_assert.h`,
  `_dspic33ck_fs_clc.{c,h}`, `_dspic33ck_pins.{c,h}`, `_dspic33ck_diag.c` and
  `_dspic33ck_diag_fast.h` files are backend-private.
- **Backend-private identifiers** inside those files: the device adapter
  `NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE`, the register-layer masks, `..._hw_*`, and the
  hot-path inlines — none of which a consumer of the public headers sees.

The tag is `_dspic33ck`, the device family this backend actually drives — a different
silicon family from dsPIC33AK (dsPIC33**C** vs dsPIC33**A**), and never abbreviated to
`_dspic33c`.

### Relationship to the dsPIC33AK HAL of the same name

[nora-hal-dspic33ak-spi-i2s-tdm](https://github.com/sulaolab/nora-hal-dspic33ak-spi-i2s-tdm)
is the same transport contract for the dsPIC33AK family. The **API is aligned 1:1** —
the system-topology model, the domain lifecycle, the diagnostics, the TDMsum profiler
and the co-clocked block all carry the same names and semantics here, which is what
lets a consumer move between families. Where the silicon forces a difference:

| | dsPIC33CK (here) | dsPIC33AK |
|---|---|---|
| **`nora_tdm_slot_t`** | a **struct of two `uint16_t` wire halves**, MS half first | a transparent `int32_t` |
| 50%-duty FS for a TDM master | **CLC1** + virtual pin **RPV0** (RP176) | CLC10 + RPV8, AK512 only |
| PPS routing | through the sibling PPS HAL (`nora_pps.h`) | inside the transport |
| physical SPI reach | SPI1/2/3 on CK256MP508 and CK1024MP710; SPI1/2 on CK64MC105 and CK256MC005 (no SPI3). `spi4()` exists for contract completeness and **always returns NULL** | SPI1..SPI4, plus a `BASE_ON_SPI34` bank |
| DMA channels available | 4 total, so **two legs** (RX+TX each) is the practical ceiling | 8 |
| board pin routing | a HAL-owned pin layer (`..._pins.{c,h}`) takes four RP numbers | supplied entirely by the board port hook |
| DFP bit positions | asserted at compile time (`..._reg_assert.h`, 27 assertions) | no counterpart |

**The slot type is the one place the contract itself differs, and it is declared, not
accidental** — see [§11](#11-the-wire-slot-type-nora_tdm_slot_t). Sharing the *name* is
not sharing the *layout*. The dsPIC33AK and dsPIC33CK fleets are **not** symmetric, and
nothing here should be read as a claim that they are.

## 1. What this HAL does

- dsPIC33CK SPI framed mode (`FRMEN=1`) used as an I2S/TDM transport.
- RX/TX ping-pong DMA, double-buffered.
- One per-instance block callback per physical SPI: `cb(src, dst, user)`.
- Selectable frame-sync waveform via `config.fs_shape` (the app states intent; the HAL
  hides FRMSYPW/FRMCNT/CLC):
  - `FS_PULSE` — short ~1-BCLK frame sync (DSP/TDM short sync).
  - `FS_50PCT` — 50%-duty FS (I2S LRCLK style). A TDM **master** gets a ~50%-duty FS
    synthesized by **CLC1** (a J-K flip-flop toggled by a half-frame FRMSYNC marker, fed
    internally via virtual pin **RPV0**) on the same FS pin the board already routed — no
    app/CLC code and no extra pin. A TDM slave receives FS as an input, so `fs_shape` is
    accepted but has no generated-waveform effect (the value is still validated and, within
    a sync domain, compared as a framing field). See
    `nora_spi_i2s_tdm_dspic33ck_fs_clc.{c,h}`.
    **`I2S` + `MASTER` + `FS_50PCT` is refused** (`ERR_UNSUPPORTED_CONFIG`): the I2S path
    uses `FRMSYPW=1`, one *wire word*, and since MODE16 that is 16 BCLK of a 64-BCLK
    (2 × 32-bit) frame = **25%, not 50%**. It was genuinely 50% while MODE32 made one wire
    word equal one slot. Rather than return success and emit the wrong duty, the HAL
    refuses; the fix is the same CLC marker route the TDM master uses and is not
    implemented, because an unverified I2S path would only be another untested claim.
    `I2S` + `SLAVE` + `FS_50PCT` stays accepted — FS is an input there, so nothing is
    generated and no duty is claimed.
- Two configure/lifecycle paths (see [§12](#12-configuration-model-summary)):
  - **System** (recommended, multi-leg): `configure_system(setups, count)` applies ALL
    legs transactionally (all-or-nothing), then `open()` + `start_all_domains()`.
  - **Single-instance**: `inst_configure(inst, cfg)` + `open()` + `inst_start(inst)`.
  `open()` takes **no role** — it derives the clock role from the committed primary leg.
  Stop a SINGLE stream with `inst_stop()`, or a SYSTEM stream with `stop_domain()` /
  `stop_all_domains()`, then `close()` to clear the shared open-state. `close()` is a
  near-no-op: it does NOT tear down board pin routing or the external clock. Plus
  `get_status` / `get_load` diagnostics (block count, deadline-miss, ISR load) — the
  arg-less ones report the primary leg.
- Optional board/clock **port** hook (`set_port()`) for pin/CLC routing and
  external-clock bring-up/readiness — the core calls only through this registered port.
- A HAL-owned **pin layer** for the family's SPI1 TDM pins
  (`nora_spi_i2s_tdm_pins_configure(&map, role)`): a board passes four RP numbers
  (BCLK/FS/SDO/SDI) and the HAL owns the directions, the PPS tokens and the role
  switch. See [§5](#5-board-pins-four-numbers-not-forty-lines).
- Per-instance SPI framed-transport health diagnostics (`SPIROV` / `SPITUR` /
  `FRMERR`, sampled once per completed RX block) — see [§6](#6-dma-and-spi-transport-health-diagnostics).
- Multi-instance: the core owns a dense logical leg table. `spi1()`…`spi4()` always mean
  literal physical peripherals, while application-semantic legs use `inst(0)`/`inst(1)`.
  `conf.h` supplies each leg's RX/TX DMA channels, geometry, and initial `SYNC_DOMAIN`;
  per-leg format / clock role come from the runtime config. The core defines the leg
  enum, ping-pong buffers, leg table, and `_DMA<rx>Interrupt` vectors in explicit C.
- Compile-time assertions that the installed DFP still places every SPI bit where this
  HAL assumes it is (`..._reg_assert.h`, included from `..._reg.h`, so they compile on
  every build).

## 2. What this HAL does NOT do

- No codec init (e.g. WM8904) — that is board/app code.
- No board pin *policy* in the core. The pin layer of §5 owns the family fact ("FS is
  the SS1 token, and it is an output when we are the master"); the board still owns the
  four RP numbers and the decision to call it. Board FS/BCLK/DATA/MCLK routing beyond
  that, and any MCLK supply, is supplied by the registered port hook. **Exception:** for
  `TDM master + FS_50PCT`, the HAL-owned CLC1 helper does touch PPS/CLC — it temporarily
  repoints the already-routed `SSx` FS pin to `CLC1OUT` (and routes `SSx`→RPV0) and
  restores it on `release()`.
- No DSP — the callback owns any processing.
- No sample-rate policy — the transport is rate-agnostic (runs at the configured BRG or
  the incoming external clock); the supported-rate set is an app concern.
- No failsafe / board-specific teardown in the core — `close()` is a near-no-op; pin and
  clock release is left to the integrator.
- No CMSIS-SAI types in the core — `ARM_SAI_*` must not appear here.
- No automatic recovery from SPI framed-transport health-flag events (`SPIROV` /
  `SPITUR` / `FRMERR`) — the HAL only records per-RX-block observations; deciding how to
  react (log, mute, restart the stream, ignore) is an app-layer policy decision.

## 3. Required project config

- The project MUST provide `nora_spi_i2s_tdm_conf.h` on the include path.
- The HAL folder ships a self-contained template: `nora_spi_i2s_tdm_conf.h_example`.
- Copy/rename the example (or supply an equivalent header) and edit the geometry
  (`NORA_TDM_SLOTS_PER_FS`, `NORA_TDM_BLOCK_FRAMES`), the logical leg count
  (`NORA_TDM_USE_SPI2`), the per-instance DMA channels
  (`NORA_TDM_SPI<n>_RX_DMA` / `_TX_DMA`), the per-leg `NORA_TDM_SPI<n>_SYNC_DOMAIN`
  defaults, `NORA_TDM_DEFINE_DMA_VECTORS`, and `NORA_TDM_SUMPROF`. `*.h_example` is
  never compiled.
- The template is self-contained (no app-config dependency). A project MAY instead
  derive the `NORA_TDM_*` macros from its own app config (the upstream project does
  exactly that); dependency is app → `conf.h` → HAL, never HAL → app.

> **`NORA_TDM_SUMPROF` (0 or 1) is mandatory** — `nora_spi_i2s_tdm.h` `#error`s if it is
> undefined. It is not defaulted on purpose: the macro gates code *out*, so an absent
> macro would evaluate to 0 in the `#if` and silently drop the TDMsum profiler. Copy the
> block from the example; `1` matches the dsPIC33AK default.

## 4. Required sibling HALs

This port needs **three** siblings; the dsPIC33AK port needs two. All three are
published for this family:

- `nora_dma` — DMA channel setup/arming (**required**). Standalone repo:
  [nora-hal-dspic33ck-dma](https://github.com/sulaolab/nora-hal-dspic33ck-dma). The
  dependency is **explicit in a header**: `nora_spi_i2s_tdm_diag.h` includes
  `nora_dma.h`, because the diagnostics and the leg table use the DMA HAL's
  `nora_dma_channel_t` / `nora_dma_trigger_t` / status types instead of raw register
  bytes. A consumer's include path must therefore reach the DMA HAL to compile the
  public headers, not only to link. The backend also uses
  `nora_dma_dspic33ck_fast.h` on its ISR path.
- `nora_high_res_timer` — compile/link sibling dependency for the load monitor.
  Runtime use is gated by `nora_high_res_timer_is_initialized()`; if the timer is not
  initialized, `get_load()` / `inst_get_load()` returns `false` and zeroes the supplied
  load struct. Standalone repo:
  [nora-hal-dspic33ck-timer](https://github.com/sulaolab/nora-hal-dspic33ck-timer) —
  on this family that counter is **SCCP1**, not Timer2.
- `nora_gpio` + `nora_pps` — the pin layer (§5) and the CLC1 frame-sync generator route
  through them; `nora_pps_find_output_rp()` is how the FS pin is discovered. Standalone
  repo: [nora-hal-dspic33ck-gpio](https://github.com/sulaolab/nora-hal-dspic33ck-gpio).
  **This is a dsPIC33CK-only dependency** — the AK port keeps its PPS handling inside
  the transport. It was moved out deliberately: the transport's private copy of the
  RP↔register map is where the non-contiguous-RP bug lived, and a second copy of a
  device fact is a second chance to get it wrong.
- The SPI register-mask helper and its DFP assertions
  (`..._dspic33ck_reg.h`, `..._dspic33ck_reg_assert.h`) ship inside this HAL folder.

An **optional debug build** (`ENA_TDM_DBG`, off by default and commented out in
`..._diag.c`) additionally pulls in `<stdio.h>`, `nora_tick_timer.h`, and a
board-supplied **`board_dbg_pins.h`** for scope pins. Nothing in a default build
requires them.

## 5. Board pins: four numbers, not forty lines

`ev88g73a_tdm_pins_init()` and `dm330030_tdm_pins_init()` were the same forty lines
twice: the same four directions, the same four PPS tokens, the same role switch. The
only difference was four RP numbers — and an RP number is a *board* fact, while "SPI1's
frame clock is the SS1 token and it is an output when we are the master" is a fact about
the *device family*. So the family fact lives here:

```c
#include "nora_spi_i2s_tdm_dspic33ck_pins.h"

static const nora_spi_i2s_tdm_pinmap_t board_tdm_pins = {
    .bclk = BOARD_TDM_BCLK_RP,   /* SCK1              */
    .fs   = BOARD_TDM_FS_RP,     /* SS1 (FRMSYNC)     */
    .sdo  = BOARD_TDM_SDO_RP,    /* SDO1, dsPIC->codec */
    .sdi  = BOARD_TDM_SDI_RP,    /* SDI1, codec->dsPIC */
};

if (!nora_spi_i2s_tdm_pins_configure(&board_tdm_pins,
                                     NORA_SPI_I2S_TDM_CLOCK_SLAVE)) {
    /* the first GPIO/PPS call that refused; pins are left partly routed */
}
```

The role is an argument, not a build-time switch, because the wiring supports both
directions on both boards: which side drives BCLK/FS is the profile's choice, not the
board's. Nothing in this file reads a board header. When the transport is later asked
for a 50%-duty FS it repoints the FS pad at `CLC1OUT` itself, so the board still knows
nothing about CLC1.

## 6. DMA and SPI transport-health diagnostics

The RX ISR preserves the raw DMA status word before resolving a completed ping-pong
half:

- `rx_dma_overrun_count` counts snapshots with `DMAINTn.OVRUNIF`.
- `rx_dma_other_irq_count` counts snapshots with neither `HALF` nor `DONE`.
- `rx_dma_last_status` exposes the latest raw snapshot. (Stored as `uint16_t` — the
  register is 16-bit on this family — and widened to `uint32_t` in the public status
  struct for source parity with the AK port.)

An OVERRUN-only interrupt therefore stays visible even though no audio block can be
delivered. RX `OVRUNIF` is the primary RX-DMA request-overrun signal; `SPIROV` may
appear downstream when RX service stalls; `SPITUR` independently reports TX FIFO
starvation, and `FRMERR` reports frame-sync health.

`IGNROV` and `IGNTUR` are deliberately hard-forced to 1 by the HAL and are **not**
caller configuration fields. This prevents a secondary FIFO flag from critical-stopping
the SPI leg and hiding the primary DMA failure. It is a continuity/containment policy,
not a claim that lost data is benign; all DMA and SPI diagnostics remain observable.

- `nora_spi_i2s_tdm_hw_sample_ack_errflags_hot(statl)` — a `static inline` in the
  backend-private `..._dspic33ck_hw.h`, not in a public header — samples `SPIxSTATL` once
  per completed RX block and returns the observed `SPIROV | SPITUR | FRMERR` mask (as
  read, before any clear).
- **Sticky-flag ownership contract.** Calling that function is how the HAL takes
  ownership of the software-clearable bits it finds set. There is exactly one ack point
  per instance, inside the RX-block ISR path, and the clear writes only the
  software-clearable mask with the observed bits zeroed — never a replay of the whole
  status word — so a clearable bit asserted after the snapshot is preserved. After the
  call, other code reading the raw status register sees those bits already acked.
  `SPITUR` is not software-clearable (hardware clears it on `SPIEN=0`), so it is only
  ever *observed*.
- `nora_spi_i2s_tdm_diag_note_errflags()` folds that mask into per-instance counters,
  read back via `get_status()` / `inst_get_status()`:
  - `err_rov_block_count` — RX blocks in which `SPIROV` was observed.
  - `err_tur_block_count` — RX blocks in which `SPITUR` was observed set.
  - `err_frm_block_count` — RX blocks in which `FRMERR` was observed.
  - `frmerr_consecutive_blocks` — length of the current run of consecutive blocks with
    `FRMERR`; reset to 0 the moment a block completes without it.
  - Each counts **RX blocks**, not raw hardware events: a flag that stays set across
    several blocks increments its counter once per block.
- **No automatic recovery policy.** The HAL counts and reports; it does not reset the
  peripheral, retry, or react.

### Engine-wide combined-occupancy profiler ("TDMsum")

The per-leg `get_load()` monitor times each RX-block ISR on its own, so leg A's peak and
leg B's peak can fall in different windows — adding the two peaks overstates the real
load. The TDMsum profiler instead reports, over one fixed common window, the peak of the
**time union** during which *any* TDM RX-block ISR was executing (overlap counted once):

- `nora_spi_i2s_tdm_tdmsum_configure(window_period_ticks)` — set the window length in
  raw high-res-timer counts (the block deadline converted to counts) and re-base the
  grid; call again whenever the deadline changes.
- `nora_spi_i2s_tdm_tdmsum_reset()` — re-base and clear peaks, keeping the window length.
- `nora_spi_i2s_tdm_tdmsum_get(&out, clear_peak)` — snapshot `max_busy_ticks` /
  `saturated_count`.

All three mask every configured leg's RX-DMA IE around the access, so the caller does
not. The measurement is TDM **active wall time**: a higher-priority non-TDM interrupt
that preempts a TDM ISR is counted inside it. The three entry points and the profiler's
ISR hooks exist only when `NORA_TDM_SUMPROF` is 1 — with 0 a reference fails at compile
time rather than silently returning a never-updated zero snapshot. The hot-path hooks
live in the backend-private `..._dspic33ck_diag_fast.h` as `static inline`, so they add
no call overhead in the ISR.

## 7. Interrupt ownership

- `NORA_TDM_DEFINE_DMA_VECTORS=1` (default): the HAL defines the RX DMA interrupt
  vectors itself (turnkey). Nothing else to wire.
- `NORA_TDM_DEFINE_DMA_VECTORS=0`: the HAL defines no vectors; the integrator owns the
  IVT and calls `nora_spi_i2s_tdm_inst_rx_isr(inst)` from their own `_DMA<rx>Interrupt`
  for each instance.

TX is interrupt-less (no TX interrupt is enabled by the transport). The CPU interrupt
enable/flag bits are driven through the DFP's single-bit aliases (`_DMAnIE` / `_DMAnIF`)
rather than a per-device `IEC`/`IFS` pointer-and-mask table.

## 8. Supported devices

Silicon facts present:

- `__dsPIC33CK256MP508__` — SPI1/2/3
- `__dsPIC33CK64MC105__` — SPI1/2 only (**no SPI3** on this part)
- `__dsPIC33CK256MC005__` — SPI1/2 only (**no SPI3** on this part)
- `__dsPIC33CK1024MP710__` — SPI1/2/3

The HAL `#error`s on any other device. Adding a new dsPIC33CK part means adding its
silicon facts in the HW layer (`..._dspic33ck_hw.{c,h}`): SPI instance count, the
`SPIxBUFL` / `SPIxCON1L` / `SPIxCON1H` / `SPIxCON2L` / `SPIxBRGL` / `SPIxIMSKL` /
`SPIxSTATL` pointers, and the DMA trigger values (as `nora_dma_trigger_t`). The CLC1
frame-sync generator `#error`s separately for a device it has no pin table for, rather
than silently using the wrong one.

`spi4()` is declared for contract completeness and **always returns NULL**: no supported
dsPIC33CK part has a fourth SPI. `spi3()` resolves only on a part with SPI3 (CK256MP508, CK1024MP710), and only if a
`conf.h` leg row targets SPI3 — on CK64MC105 nothing should target it, and it reads NULL
by configuration rather than by an enforced device check.

**Two legs is the practical ceiling on this family**, because a leg needs an RX and a TX
channel and the DMA controller has four channels in total.

## 9. Tested envelope

**How to read this.** These HALs are built for evaluation, FAE demos and architecture
experiments, so exhaustive per-function coverage was never the goal — there is no unit-test
suite, and the evidence is integration testing on real hardware: the code ran as
part of a working
audio system, and that system's own observations are the measurement. What follows is
therefore split by **what kind of evidence exists**, in three tiers used here and in the
sibling repositories:

- **Integration-verified** — it ran in the working system, and something observable would
  have broken if it had not.
- **Hardware-observed, not a matrix** — it worked in the configuration(s) actually run;
  other combinations are untried, not known-good.
- **Compiled, not executed** — no caller, or excluded from the build that runs, or dropped
  from the image by the linker.

- Devices: dsPIC33CK64MC105 (EV88G73A Curiosity Nano), dsPIC33CK1024MP710 (EV97U97A)
  and dsPIC33CK256MP508 (DM330030)
- Compiler: XC-DSC v3.31.01
- DFP: dsPIC33CK-MC_DFP 1.10.386 / dsPIC33CK-MP_DFP 1.15.423 (MP710: 1.16.521) or compatible

Measured on hardware (EV88G73A + WM8904, 2026-08-11):

- A live codec passthrough with the block callback running: `max = 47.2 µs (7.0 %)` of a
  `671.2 µs` block deadline, **`miss = 0` across more than 137,000 blocks**, and no
  diagnostic counter moved off zero for the duration.
- Stream restart, path switching between several callbacks (chain / gain / copy / mute /
  tone) at runtime, and recovery of the audio path after three forced CPU traps and a
  cold power cycle.
- The codec input was verified live (per-slot peak reporting at noise-floor level), and
  the DMA controller itself was proved independently by a RAM→RAM self-test each boot.
- Word width: **32-bit** (`word_bits` — only 32 is validated).
- `NORA_TDM_SLOTS_PER_FS` = **8** (TDM8). No configuration that was run
  overrides it, so 8 is the only geometry that has ever run.
- **Slave** (external BCLK/FS from the codec) is the main tested path.

**The master role is integration-verified too, including the CLC1 50%-duty FS.** A build
with the dsPIC as TDM master (WM8904 as slave, MCLK supplied by the dsPIC at the board's
`A-extMCLK` jumper position) ran the same passthrough. Six independent observations, not
one:

1. `passthrough running (TDM8/32-bit MASTER, WM8904 slave)` — the engage branch for
   `MASTER + TDM + FS_50PCT` is **not optional** in that build and its only alternative
   exit is `goto fail`, so the banner cannot be printed unless
   `..._fs_clc_engage()` returned OK.
2. Block count climbed past **771,000 blocks with `miss = 0`** while the dsPIC was the
   *only* source of FS and BCLK.
3. The reported block deadline moved from 671.2 µs to **655.3 µs — the ratio is exactly
   48000/48828**, i.e. the transport really ran on our own BRG-derived rate rather than
   still on the codec's.
4. The 48 kHz-only audio feature refused itself with an explicit rate mismatch, which is
   the same rate fact reported by a second, independent path.
5. **The codec returned live ADC data** (`CODEC-IN peak slot0=5898 slot1=5875`,
   `active slots=01......`) — the WM8904 was clocked as a *slave* off our frame sync, which
   is impossible unless a real ~50%-duty FS was present on the pad. This is the strongest
   of the six.
6. An operator listened to music through the path, and mute worked. A frame clock that is
   present but skewed or jittered is audible.

The SYSTEM/domain lifecycle gates were then re-run in this role and answered **5/5 as
specified**, with the stream still running afterwards. Separately, the validating project's own parity record lists `FS_PULSE` and 50%-duty FS are both listed as verified **with a scope**,
along with the slot landmark (value in slots 1-2, six silent) and the LOW-half-word-first
wire order of §11.

**Hardware-observed, not a matrix.** Exactly these combinations ran, all of them
`format = TDM`, `slots_per_fs = 8`, `word_bits = 32`, on one part (EV88G73A):

| role | `fs_shape` | evidence |
|---|---|---|
| SLAVE | `FS_PULSE` | the codec-master passthrough soak above (>137,000 blocks, `miss = 0`) |
| MASTER | `FS_50PCT` | the six observations above (>771,000 blocks) + scope |
| MASTER | `FS_PULSE` | the master loopback with the CLC out of the path, + scope |

**`format = I2S` has never been configured on hardware, in either role** — every caller in
every caller sets `FORMAT_TDM`, so the I2S branch of `hw_apply_config()` is
compile-verified only. The full `slots_per_fs` × `word_bits` × `format` × role × `fs_shape`
product was not swept: a combination not in the table is untried rather than known-good.

Measured on hardware (EV97U97A / CK1024MP710 + WM8904, 2026-10-13): SPI1 as **SLAVE**,
`FS_PULSE`, TDM8, 32-bit slots, 48 kHz, codec master. With the block callback running:
`miss = 0` across more than 202,000 blocks, sampled `max` about 373 µs of a 667 µs block.
The master role, `FS_50PCT` and SPI2/SPI3 have not run on this part.

**Compiled, not executed:**

- `CK256MP508_DM330030` is **compile-only**, so this HAL's SPI3 facts
  and its three-leg possibilities compile and have not run on silicon.
- The co-clocked dual-codec block (`inst_tx_fill_mirror()` and the phase probes) has **no
  caller in this build**. It is present because the contract requires every backend whose
  silicon can co-clock two legs to implement all four entry points under the same names;
  with per-function sections and section GC, an unused set costs zero bytes.

## 10. Portability boundary

`nora_spi_i2s_tdm.h` **is** the NORA transport contract — the boundary at which
portability is defined. Do not add a second, smaller portability facade above it: an
application-facing stream API layered on this transport can only ever offer the
intersection of what each backend supports, and it doubles the public surface for one
peripheral.

Other silicon backends adapt **below** this contract, not in front of it. A capability a
backend lacks is exposed explicitly — as something the caller can query, or as an
explicit unsupported result — rather than hidden by narrowing the shared API or emulated
behind the caller's back. `spi4()` returning NULL and `fs_shape` being accepted-but-inert
for a slave are both examples.

Sharing the contract does **not** require every backend to compile against a
byte-identical public header; contract agreement and header identity are separate
questions, and only the first is promised here. §11 is the concrete case.

## 11. The wire slot type `nora_tdm_slot_t`

On this family the DMA element **is** a 16-bit wire word, so a 32-bit audio slot is two
of them and the most-significant half goes out first:

```c
typedef struct {
    uint16_t wire[2];   /* wire[0] first on the wire (MS half), wire[1] second */
} nora_tdm_slot_t;
```

Both families define the same spelling, both assert `sizeof == 4`, and **only the layout
differs** — dsPIC33AK's `nora_tdm_slot_t` is a plain `int32_t`, which is the honest
description of an AK DMA element. Sharing the name is not sharing the layout.

Consequences, all deliberate:

- `dst[i] = sample;` for an `int32_t` sample is a **compile error** here rather than
  silently-wrong audio. That is the whole point of using a struct.
- A raw passthrough needs no conversion at all: `dst[i] = src[i]` copies a slot already
  in wire order.
- Conversion costs about two instructions per sample **when folded into a store the DSP
  already performs**. A separate conversion pass over the block costs 3–4× more.

Three rules for code meant to move between families — contract, not style:

1. **Go through the accessors** (`nora_tdm_slot_encode_s32()`,
   `nora_tdm_slot_decode_s32()`, `nora_tdm_slot_scale_q15()`). This family *enforces*
   it; dsPIC33AK cannot detect a violation, because its typedef is transparent — so
   portable code written there compiles clean and breaks on arrival here.
2. **A slot is not a byte sequence.** Both families are exactly 4 bytes and both assert
   it, so `memcpy`ing slot buffers between families, persisting them, or reinterpreting
   them as `int32_t` **compiles everywhere and is wrong**. Cross-family transfer,
   storage and reinterpretation of the raw bytes are outside this contract.
3. **Fold the conversion into the store/load the DSP already performs**; do not add a
   conversion pass. On AK encode/decode are the identity and cost nothing, which is
   exactly why an author there cannot see the trap.

`nora_tdm_slot_scale_q15()` is **exact**, not an approximation, and its cost is
constant: on this 16-bit core a 16×16 multiply is one cycle and `>>16` is free (it is
just naming the high word), whereas a Q31 gain forces a 64-bit multiply helper call
measured at ~88 cycles per sample. Unity (`0x8000`) is bit-exact and zero is a true
digital zero, with no branch either way.

## 12. Configuration model (summary)

Two ways to configure, both ending in `open()` → start:

- **System (transactional, recommended).** `configure_system(setups, count)` takes one
  leg setup per leg (`{ stream, sync_domain }`) and is all-or-nothing. A
  side-effect-free PREFLIGHT rejects the whole call — touching no leg — if any leg is
  running or outside the wire-format envelope, if a sync domain holds more than one
  clock MASTER, if two legs sharing a sync domain disagree on the frame interpretation
  (format / `word_bits` / slots / `block_frames` / SPIFE / CKP / CKE / `fs_shape`), or if
  any `sync_domain` is out of the trackable range. Only after a clean preflight are all
  legs committed together, so there is never a half-configured mix.
- **Single-instance.** `inst_configure(inst, cfg)` validates and stores one leg's config;
  use it with `open()` + `inst_start(inst)` for a single-leg driver.

The two paths establish a mutually-exclusive **config-ownership mode** — a property of
the committed configuration, independent of the open/close lifecycle (`close()` does not
reset it): `inst_configure()` → **SINGLE**, in which the per-leg API is legal only on the
**primary** leg; `configure_system()` → **SYSTEM**, in which the whole-system domain API
is legal. A call from the wrong family returns `ERR_CONFIG_MODE`.

`open()` is idempotent; `close()` and `set_port()` return `bool` and reject while a leg
is running (or, for `set_port()`, while open). The start paths re-check the
clock-readiness gate immediately before arming, so a source that drops between `open()`
and start fails the start (`ERR_CLOCK_NOT_READY`). `open()` derives the clock role from
the committed **primary** leg and fails (`ERR_NOT_CONFIGURED`) if the primary is
unconfigured; a port hook that also routes a secondary leg reads that leg's committed
role via `inst_get_setup()`. `start_all_domains()` starts each sync domain once,
releasing co-clocked members' `SPIEN` back-to-back (slaves first, clock master last) so
they latch one frame edge.

### Block-callback contract (summary)

Register the callback with `set_block_callback()` **before the leg is started** —
`inst_start()` in SINGLE mode, `start_domain()` / `start_all_domains()` in SYSTEM mode,
where `inst_start()` is itself refused with `ERR_CONFIG_MODE`. The call has **no**
config-mode gate of its own (a callback is per-instance state, not part of the
SINGLE/SYSTEM split), so it is legal on any leg in either mode. Do not
swap/clear it while running (an identical re-register is an idempotent no-op). When the
callback runs, **`src` and `dst` are both non-NULL** — if the core cannot resolve the RX
or TX half it skips that block instead of calling the callback, so the callee never
NULL-checks. With no callback, that instance runs no DSP path (its zeroed TX half stays
silent). `src`/`dst` are **wire slots**, not `int32_t` samples — see §11.

### Diagnosing a failed call

The bool-returning calls — `set_port()`, `open()`, `close()`, `inst_configure()`,
`configure_system()`, `inst_start()`, `inst_stop()`, `start_domain()`,
`start_all_domains()`, `stop_domain()`, `stop_all_domains()`, `set_block_callback()` —
collapse several causes into one `false`. On `false`,
`nora_spi_i2s_tdm_get_last_error()` returns the most specific reason
(`nora_spi_i2s_tdm_error_t`: `ERR_BAD_ARGUMENT`, `ERR_BAD_INSTANCE`,
`ERR_NOT_CONFIGURED`, `ERR_NOT_OPEN`, `ERR_ALREADY_OPEN`, `ERR_ALREADY_RUNNING`,
`ERR_CONFIG_MODE`, `ERR_TOPOLOGY`, `ERR_UNSUPPORTED_CONFIG`, `ERR_PIN_CONFIG`,
`ERR_DMA_CONFIG`, `ERR_CLC`, `ERR_CLOCK_INIT`, `ERR_CLOCK_NOT_READY`). This is a debug
aid only — stream health (deadline misses, block counts) lives in `get_status()`, not
here.

## Files

```text
src/
  nora_spi_i2s_tdm.h                        public contract (+ the wire slot type)
  nora_spi_i2s_tdm_diag.h                   public diagnostics counters/snapshots
  nora_spi_i2s_tdm_conf.h_example           self-contained conf.h template (never compiled)
  nora_spi_i2s_tdm_dspic33ck.c              CK backend: core engine, leg table, DMA vectors
  nora_spi_i2s_tdm_dspic33ck_hw.{c,h}       silicon layer: SPI instance facts, register/DMA ops
  nora_spi_i2s_tdm_dspic33ck_reg.h          SPI framed-mode register masks (L/H split)
  nora_spi_i2s_tdm_dspic33ck_reg_assert.h   DFP bit-position assertions (27)
  nora_spi_i2s_tdm_dspic33ck_fs_clc.{c,h}   CLC1 50%-duty FS generator (TDM master)
  nora_spi_i2s_tdm_dspic33ck_pins.{c,h}     the family's SPI1 TDM pin routing
  nora_spi_i2s_tdm_dspic33ck_diag.c         out-of-line diagnostics
  nora_spi_i2s_tdm_dspic33ck_diag_fast.h    per-block ISR hot hooks (static inline)
```

## Notes

- This repository does not include Microchip DFP header files.
- A CMSIS-SAI wrapper would be a layer **above** this HAL, not part of it; `ARM_SAI_*`
  types must not appear here. No dsPIC33CK SAI wrapper is published.
- **Do not confuse the two layers' event names.** This HAL's `err_rov_block_count` /
  `err_tur_block_count` count the SPI **hardware** flags `SPIROV` / `SPITUR`, sampled
  once per RX block at the transport layer. A CMSIS wrapper's `rx_overflow` /
  `tx_underflow` are its **own** software buffer-semantics events one layer up. The names
  read alike; the failure modes are different.

* Sibling repositories for this family:
  [dma](https://github.com/sulaolab/nora-hal-dspic33ck-dma) ·
  [timer](https://github.com/sulaolab/nora-hal-dspic33ck-timer) ·
  [gpio](https://github.com/sulaolab/nora-hal-dspic33ck-gpio) ·
  [clock](https://github.com/sulaolab/nora-hal-dspic33ck-clock) ·
  [i2c](https://github.com/sulaolab/nora-hal-dspic33ck-i2c) ·
  [uart](https://github.com/sulaolab/nora-hal-dspic33ck-uart)

## Source snapshot

`src/` is byte-identical to the private development tree at `0ff5854`. Re-publish from that revision or later, not from an earlier snapshot of this repository.

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.
