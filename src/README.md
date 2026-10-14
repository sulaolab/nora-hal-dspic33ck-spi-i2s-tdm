# dsPIC33CK SPI/I2S/TDM transport HAL — sources

One public contract, one public diagnostics header, and a dsPIC33CK backend split into
layers. Full documentation is in the [repository README](../README.md); this file is the
ownership boundary and the layering.

## Layers

| file | layer | may a consumer include it? |
|---|---|---|
| `nora_spi_i2s_tdm.h` | the transport **contract** — lifecycle, config, the wire slot type, callbacks | **yes** |
| `nora_spi_i2s_tdm_diag.h` | public diagnostics: block counters, deadline miss, ISR load, SPI health | **yes** |
| `nora_spi_i2s_tdm_conf.h_example` | template for the `nora_spi_i2s_tdm_conf.h` **you** supply | copy it; it is never compiled |
| `nora_spi_i2s_tdm_dspic33ck.c` | core engine: leg table, ping-pong buffers, DMA vectors | no |
| `nora_spi_i2s_tdm_dspic33ck_hw.{c,h}` | silicon facts: SPI instance table, register and DMA ops | no |
| `nora_spi_i2s_tdm_dspic33ck_reg.h` | SPI framed-mode register masks (L/H split) | no |
| `nora_spi_i2s_tdm_dspic33ck_reg_assert.h` | DFP bit-position static assertions; included from `..._reg.h` | no |
| `nora_spi_i2s_tdm_dspic33ck_fs_clc.{c,h}` | CLC1 50%-duty frame-sync generator | no |
| `nora_spi_i2s_tdm_dspic33ck_pins.{c,h}` | the family's SPI1 TDM pin routing, given four RP numbers | **yes** — a board calls it |
| `nora_spi_i2s_tdm_dspic33ck_diag.c` | out-of-line diagnostics bodies | no |
| `nora_spi_i2s_tdm_dspic33ck_diag_fast.h` | per-block ISR hot hooks, `static inline` | only on a measured ISR path |

Everything tagged `_dspic33ck` other than the pin layer is **backend-private**: it
exposes SFRs, and including it is a declaration that the includer is backend-aware.

## Ownership boundary

What this HAL owns:

- The SPI framed-mode registers of each configured leg, and that leg's RX/TX DMA channel
  programming (through the DMA HAL — it does not write DMA SFRs itself).
- The RX DMA interrupt vectors, when `NORA_TDM_DEFINE_DMA_VECTORS` is 1.
- `CLC1` and virtual pin `RPV0`, but **only** while a TDM master with `FS_50PCT` holds
  them; ownership is per instance/clock-domain and a second claimant is refused.
- The four SPI1 pin directions and PPS tokens, when a board calls
  `nora_spi_i2s_tdm_pins_configure()`.

What the consumer owns, and must supply:

- **`nora_spi_i2s_tdm_conf.h`** on the include path — geometry, per-leg DMA channels,
  sync domains, `NORA_TDM_DEFINE_DMA_VECTORS`, and `NORA_TDM_SUMPROF` (mandatory).
- **Three sibling HALs on the include path**: `nora_dma`, `nora_high_res_timer`, and
  `nora_gpio`/`nora_pps`. The first is needed to *compile the public headers*, not only
  to link.
- **The clock tree**, the codec, the sample-rate policy, and the DSP inside the block
  callback.
- **The board's RP numbers**, and any routing beyond the four SPI1 pins — through the
  `set_port()` hook.
- **A reaction policy** for the SPI health counters. This HAL counts; it never resets,
  retries, or mutes.

## Non-goals

- No DSP, no codec driver, no sample-rate policy, no failsafe teardown.
- No `ARM_SAI_*` types — a CMSIS-SAI wrapper is a layer above, not part of this HAL.
- No second portability facade above `nora_spi_i2s_tdm.h`: that header *is* the boundary
  at which portability is defined.
- No automatic recovery from `SPIROV` / `SPITUR` / `FRMERR`.

## References in the sources that point outside this repository

Some comments in these sources cite the project they were developed and validated in.
They are left in place deliberately — the provenance of a silicon fact is worth more
than a tidy comment:

- **The CK silicon findings note** is cited by absolute URL to its one public copy,
  [`docs/ck_silicon_findings.md` in `dspic33ck-hal-lab`](https://github.com/sulaolab/dspic33ck-hal-lab/blob/main/docs/ck_silicon_findings.md),
  so those eleven references resolve from here. The note is maintained in that one
  place and is never duplicated into this repository, which is why the links are
  absolute rather than repository-relative. It holds the measured evidence for MODE16,
  the wire-half order and the `___muldi3` cost; every fact the comments depend on is
  also stated inline, so nothing is unreadable without it. The numbered *"audit defect
  N"* / *"phase N"* labels index that note and the order the lifecycle grew in.
- **App-layer symbols** named as examples of who supplies a policy —
  `audio_app_board_get_default_config()`, `APP_SAMPLE_RATE_IS_SUPPORTED`,
  `ENA_USB_AUDIO_IN`, `wm8904_audio_stop()` — are the upstream integration's names, not
  part of this HAL's contract and not required by it. They mark the boundary, not a
  dependency.
- **Board facts** such as the external-clock detect on `RB15` / `CN` describe the board the
  transport was validated against. This HAL only defines the *event* the port hook feeds
  it; a board with no such detect reports `CLOCK_EVENT_NONE` forever.

## The one thing to read before writing a callback

`src`/`dst` are `nora_tdm_slot_t` — on this family a **struct of two 16-bit wire halves,
most-significant first**, not an `int32_t`. `dst[i] = sample;` is a compile error here by
design. Use `nora_tdm_slot_encode_s32()` / `_decode_s32()` / `_scale_q15()`, and fold the
conversion into a store the DSP already performs rather than adding a pass over the
block. A raw passthrough needs no conversion at all (`dst[i] = src[i]`). The dsPIC33AK
port shares the *name* and the accessor vocabulary but not the layout — see §11 of the
[repository README](../README.md) before moving code between the families.
