#ifndef NORA_SPI_I2S_TDM_H
#define NORA_SPI_I2S_TDM_H

// Device identity (the NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE tag and its unsupported-device
// #error) lives in the backend silicon header nora_spi_i2s_tdm_dspic33ck_hw.h: the names are
// chip-tagged, and this file is the family-neutral contract.
//
//===========================================================
// nora_spi_i2s_tdm.h (contract) + nora_spi_i2s_tdm_dspic33ck.c (CK backend) +
// nora_spi_i2s_tdm_dspic33ck_reg.h (register layer) provide a reusable
// dsPIC33CK SPI framed-mode I2S/TDM transport HAL. It owns SPI framed-mode setup,
// RX/TX DMA ping-pong buffers, per-instance block callbacks, lifecycle, status,
// deadline-miss, and load diagnostics.
//
// It does not own DSP, codec setup, sample-rate policy, app config, or CMSIS-SAI buffer
// semantics. External-clock bring-up/readiness is reached only through the registered
// board/clock port hook (set_port()).
//
// PPS/CLC is NOT entirely outside this HAL, and saying so would be wrong in two places:
//   - the pin layer (nora_spi_i2s_tdm_pins_configure) owns the PPS tokens and directions
//     of the family's four SPI1 TDM pins; the board supplies only the RP numbers, and
//     calling it at all is the board's choice.
//   - the CLC 50%-FS generator (nora_spi_i2s_tdm_dspic33ck_fs_clc.*) owns CLC1 and virtual
//     pin RPV0 outright, and for a TDM master with FS_50PCT it repoints the already-routed
//     FS pad from FRMSYNC to CLC1OUT while streaming (restored on release).
// What the HAL does not own is board pin POLICY: which pads, MCLK supply, and any routing
// beyond those four pins stay with the board / the port hook.
//
// Compile-time stream geometry and topology come from a project-supplied
// nora_spi_i2s_tdm_conf.h. The HAL core only depends on NORA_TDM_*
// macros from that header -- it does not read app symbols directly. Instance
// count and shape are fixed in the core, which defines its leg enum, per-instance
// buffers, leg table and DMA vectors explicitly; conf.h supplies the per-leg
// values they are keyed off (NORA_TDM_USE_SPI2, NORA_TDM_SPIn_RX/TX_DMA,
// NORA_TDM_SPIn_SYNC_DOMAIN) and the stream geometry.
//
// Supported-device limitation: the silicon-facts paths currently cover
// __dsPIC33CK256MP508__ (SPI1/2/3) and __dsPIC33CK64MC105__ (SPI1/2 only, no
// SPI3 on this part); other parts need their facts added in
// nora_spi_i2s_tdm_dspic33ck_hw.{c,h}. Sibling dependencies are
// nora_dma (required) and nora_high_res_timer (compile/link dependency
// for the load monitor, runtime-gated via is_initialized()).
//===========================================================

//===========================================================
// INCLUDES
//===========================================================
#include <stdint.h>
#include <stdbool.h>
#include "nora_spi_i2s_tdm_conf.h"   // HAL compile-time config (geometry/topology); exposes NORA_TDM_* to consumers

// NORA_TDM_SUMPROF gates code OUT, so an absent macro would evaluate to 0 in the #if below and
// silently drop the profiler from a project whose conf.h predates it. Demand it explicitly:
// copy the block from nora_spi_i2s_tdm_conf.h_example (default 1 = the AK-side default).
#ifndef NORA_TDM_SUMPROF
#error "nora_spi_i2s_tdm_conf.h must define NORA_TDM_SUMPROF (0 or 1) -- see nora_spi_i2s_tdm_conf.h_example."
#endif


//===========================================================
// Definition
//===========================================================


//===========================================================
// Enum & Struct typedef
//===========================================================

// Block-ISR load/time monitor: execution-time stats of one instance's RX-block ISR,
// in raw timer counts and 0.1us units. (Named after the load it measures, not a fixed
// DMA channel -- each instance's RX DMA channel is configurable in conf.h.)
typedef struct
{
    uint32_t last_count;
    uint32_t min_count;
    uint32_t max_count;
    uint32_t event_count;

    // us10 means 0.1 us unit.
    // Example: 1234 means 123.4 us.
    uint32_t last_us10;
    uint32_t min_us10;
    uint32_t max_us10;
} nora_spi_i2s_tdm_load_t;


// TDM-active COMBINED-occupancy profiler snapshot (engine-wide "TDMsum").
//
// The per-instance load monitor above times EACH RX-block ISR on its own, so leg A's peak
// and leg B's peak can occur in DIFFERENT time windows -- adding the two peaks overstates the
// real CPU load. This snapshot instead reports the peak, over any single fixed common window,
// of the TIME UNION during which ANY TDM RX-block ISR was executing (SPI1 and SPI2 overlap
// counted once, never double-added). window_period_ticks is the common window length in raw
// high-res-timer counts (derived from the same block deadline the per-leg %/margin uses);
// max_busy_ticks is the largest single-window union since the last clear; saturated_count is
// the number of windows that were fully (100%) occupied since the last clear.
//
// NOTE: this is TDM ACTIVE WALL TIME. Because a higher-priority non-TDM interrupt (e.g. the
// ADC or UART-TX vectors, which sit above PRIO_TDM_DMA) can preempt a TDM ISR, any time spent
// in such a nested non-TDM interrupt is included in the measured TDM busy time. Excluding it
// would require instrumenting every ISR and is intentionally out of scope here.
//
// TIME BASE, and why it is not in this struct: the counts are this family's high-res timer,
// which is SCCP1 here and Timer2/3 on dsPIC33AK -- dsPIC33CK has no T2CON at all. That
// difference is a BACKEND fact and stays out of the public contract: a caller asks
// nora_high_res_timer_is_present() / _is_initialized(), and an absent time base makes
// _tdmsum_get() return false rather than removing a declaration.
typedef struct
{
    uint32_t window_period_ticks;   // common window length (raw high-res-timer counts); 0 = not configured
    uint32_t max_busy_ticks;        // peak single-window TDM-union occupancy since the last clear
    uint32_t saturated_count;       // windows fully (>=100%) occupied since the last clear
    bool     initialized;           // a valid window has been configured
} nora_spi_i2s_tdm_tdmsum_t;



//===========================================================
// Runtime SPI/I2S/TDM stream configuration.
//
// configure() validates and stores this configuration while stopped. start()
// applies the stored configuration to the SPI peripheral and DMA engine after the
// platform pin/clock port hooks have completed successfully. Seed it from the
// platform helper (audio_app_board_get_default_config) or build it directly. Field
// comments map to the SPIxCON1 bit each one drives.
//===========================================================

// Frame format. Selects FRMCNT + the conventional FRMPOL used today.
//
// FRMCNT counts SERIAL WIRE WORDS, and the wire word is 16 bits while an audio slot is 32,
// so one slot is TWO wire words. Hence:
//   I2S  (2 slots) : 4 wire words  -> FRMCNT=010, FRMPOL active-low   (was ENA_FRMT_I2S)
//   TDM8 (8 slots) : 16 wire words -> FRMCNT=100, FRMPOL active-high
// Slot counts above 16 cannot be framed: 32 slots would need 64 wire words and FRMCNT
// stops at 32.
typedef enum {
    NORA_SPI_I2S_TDM_FORMAT_I2S = 0,
    NORA_SPI_I2S_TDM_FORMAT_TDM = 1,
} nora_spi_i2s_tdm_format_t;

// Bit-clock / frame-sync role.
//   SLAVE  : MSTEN=0, FRMSYNC=1 (FS input)   -- external BCLK/FS
//   MASTER : MSTEN=1, FRMSYNC=0 (FS output)  -- self-clocked (starter target)
typedef enum {
    NORA_SPI_I2S_TDM_CLOCK_SLAVE  = 0,
    NORA_SPI_I2S_TDM_CLOCK_MASTER = 1,
} nora_spi_i2s_tdm_clock_role_t;

// External frame-sync (FS/LRCK) waveform shape. This is the user-facing INTENT; the HAL
// picks the hardware mechanism, so the application never deals with FRMSYPW/FRMCNT/CLC:
// (FRMCNT below is stated in slots for readability; hw.c scales it to wire words -- 2 per
// 32-bit slot -- before encoding. See the format enum.)
//   FS_PULSE   : short frame sync, one BCLK wide at the frame start (DSP/TDM "short sync").
//                FRMSYPW=0, FRMCNT=slots_per_fs. No CLC.
//   FS_50PCT   : 50%-duty FS, I2S LRCLK style.
//                - I2S + MASTER: ** REJECTED by configure() ** with
//                  ERR_UNSUPPORTED_CONFIG. FRMSYPW=1 is one WIRE WORD wide; in MODE16 that
//                  is 16 BCLK out of a 64-BCLK (2 x 32-bit) frame = 25%, not 50%. The CLC
//                  half-frame-marker route used by the
//                  TDM master case already uses (FRMSYPW=0, marker every 2 wire words = 1
//                  slot); it is NOT implemented, because the current target is TDM8 and an
//                  unverified I2S path would be another untested claim. So rather than
//                  return success and then emit 25%, this combination is refused -- a
//                  request for a 50%-duty FS that the HAL cannot generate is answered as
//                  unsupported, not as done. I2S FS_PULSE and all TDM shapes are unaffected.
//                - I2S + SLAVE: accepted. FS is an INPUT, so nothing is generated and no
//                  duty is claimed; FRMSYPW=1 describes how the incoming FS is read, which
//                  is why fs_shape is still compared as a framing field inside a sync domain.
//                - TDM (>=4 slots), MASTER: the SPI emits a 1-BCLK half-frame marker
//                  (FRMSYPW=0, FRMCNT=slots_per_fs/2) that a CLC toggles into a 50%-duty FS
//                  on the same FS pin. The HAL owns CLC1 + virtual pin RPV0 (see
//                  nora_spi_i2s_tdm_fs_clc.*).
//                - TDM SLAVE: FS is an INPUT, so fs_shape is accepted but has no
//                  generated-waveform effect (treated as normal slave framing). The CLC
//                  50%-duty FS is generated only in master mode.
// NOTE: there is intentionally no "one-word-wide TDM" shape -- a word-wide TDM pulse is a
// niche non-50% long-frame sync and was dropped in favor of these two common intents.
typedef enum {
    NORA_SPI_I2S_TDM_FS_PULSE = 0,   // short frame sync, ~1 BCLK (FRMSYPW=0)
    NORA_SPI_I2S_TDM_FS_50PCT = 1,   // 50%-duty FS (TDM master: via CLC1; I2S master: rejected)
} nora_spi_i2s_tdm_fs_shape_t;

typedef struct {
    nora_spi_i2s_tdm_format_t format;          // I2S vs TDM (FRMCNT/FRMPOL)
    nora_spi_i2s_tdm_clock_role_t clock_role;   // master vs slave (MSTEN/FRMSYNC)
    uint8_t  slots_per_fs;                          // NORA_TDM_SLOTS_PER_FS: I2S=2 / TDM8=8
                                                    // TDM accepts 4/8/16 (not 32 -- see format enum)
    uint8_t  word_bits;                             // audio slot width. Only 32 is validated.
                                                    // NOT the SPI transfer unit: the wire runs
                                                    // MODE16 and a 32-bit slot is two wire words.
    nora_spi_i2s_tdm_fs_shape_t fs_shape;      // FS waveform intent (see enum). HAL derives
                                                    // FRMSYPW/FRMCNT and engages CLC1 as needed.
    uint16_t block_frames;                          // NORA_TDM_BLOCK_FRAMES: frames per ping/pong half
    uint32_t brg;                                   // SPIxBRG (master only; ignored as slave)
    // MCLKEN: selects the SPI BRG clock source (reference clock/REFO vs FP). It does NOT
    // enable an external MCLK output -- this HAL cannot emit an MCLK at all. (Read
    // "CLKGEN9 reference" here until 2026-08-09; that is AK vocabulary, not CK.)
    bool     mclk_enable;
    bool     fs_coincides_first_bclk;               // SPIFE: 1=no delay, 0=1-bit delayed (ENA_1_BIT_DELAY)
    bool     bclk_idle_high;                        // CKP
    bool     bclk_change_on_active_to_idle;         // CKE
    // NOTE: IGNROV and IGNTUR are NOT exposed here on purpose. Continuous DMA audio keeps both set
    // so a secondary FIFO error cannot critical-stop the SPI leg and hide the primary failure.
    // This is a continuity/containment policy, NOT a claim that data loss is benign. RX
    // DMAINTn.OVRUNIF is captured separately as the RX-DMA request-overrun signal. SPIROV may
    // follow stalled RX service; SPITUR independently reports TX starvation; FRMERR tracks framing.
} nora_spi_i2s_tdm_config_t;

// Clock-change event reported by the slave's external-clock detector
// (board RB15 / CN). STOPPED = the external bit/frame clock has stopped (e.g. the
// source is switching sample rate) -> the app should mute + stop. RESUMED = the
// clock is back -> the app should measure the rate, reconfigure, then restart.
// NONE = nothing pending. Consumed (read-and-clear) via
// nora_spi_i2s_tdm_consume_clock_event(); always NONE when the board has no
// external-clock detect (e.g. ENA_USB_AUDIO_IN undefined).
typedef enum {
    NORA_SPI_I2S_TDM_CLOCK_EVENT_NONE = 0,
    NORA_SPI_I2S_TDM_CLOCK_EVENT_STOPPED, // external clock stopped (rate switch begun)
    NORA_SPI_I2S_TDM_CLOCK_EVENT_RESUMED, // external clock back
} nora_spi_i2s_tdm_clock_event_t;

//===========================================================
// TDM SLOT -- the element type of the block callback's buffers, and of the TX fill pointer.
//
// One name, one representation per family, and the representation belongs to the backend:
//   AK: int32_t, transparent. MODE32 gives a single 32-bit SPIxBUF and 32-bit DMA
//       elements, so one sample IS one DMA element and there is nothing to pack.
//   CK: struct { uint16_t wire[2]; } -- this family, defined below. The DMA element here
//       is a 16-bit wire word; the struct is what keeps that visible.
//
// This is the DMA contract's typed-value rule (nora_dma_status_t) applied one level up: the
// TYPE is shared, the LAYOUT is not, and a consumer touches the value only through the
// accessors below. It is not a portability facade -- no extra header, no conversion layer,
// no copy, no runtime cost. What it buys is that the block-callback signature is textually
// identical in both families while a portable consumer that writes `dst[i] = sample` still
// FAILS TO COMPILE here, at build time, loudly.
//
// The DMA buffer element on THIS family: ONE TDM SLOT IN WIRE ORDER
//
// A 32-bit audio slot does NOT live in the DMA buffers as an int32_t, and this type
// exists to make that impossible to forget. The SPI uses MODE16, so its data
// port is the 16-bit SPIxBUFL and a 32-bit slot is TWO independent wire words. The DMA
// walks the buffer in ascending address order, and this core is little-endian -- so an
// int32_t in a DMA buffer transmits its LOW half FIRST, which is backwards from the
// MSB-first convention every TDM/I2S wire uses. A loopback can mask this because RX
// receives the same ordering that TX emitted.
//
// So the buffer element names the wire, not the host:
//
//     wire[0] is transmitted (and received) FIRST -- the MOST significant half
//     wire[1] is transmitted (and received) SECOND
//
// Consequences, all of them deliberate:
//   - `dst[i] = sample;` for an int32_t sample is now a COMPILE ERROR rather than
//     silently-wrong audio. That is the whole point of using a struct here.
//   - Converting costs ~2 instructions per sample when folded into an existing store
//     (measured with -Os). Do NOT add a separate
//     swap pass over the block -- that costs 3-4x more. Encode at the point where the
//     DSP already stores its result, and decode where it already loads its input.
//   - A raw passthrough needs NO conversion at all: `dst[i] = src[i]` copies a slot
//     already in wire order.
//
// The STRUCT is CK-specific and is NOT ported to dsPIC33AK/sonora: `int32_t` is the honest
// description of an AK DMA element, and wrapping it in a two-half struct there would invent
// a wire order the silicon does not have. What IS shared is the NAME `nora_tdm_slot_t` and
// the encode/decode/scale accessor vocabulary below -- both families define the same
// spellings, both assert sizeof == 4, and only the layout differs. Sharing the name is not
// sharing the layout.
//
// THREE RULES FOR CODE THAT IS MEANT TO MOVE BETWEEN FAMILIES. All three are contract,
// not style:
//
//  1. Go through the accessors. This family enforces that -- `dst[i] = sample` is a compile
//     error here, which is the whole point of using a struct. It is AK that cannot detect a
//     violation, because its typedef is transparent: portable code written there compiles
//     clean and breaks on arrival here. So the rule is stated for both sides, and it is this
//     build that catches it. (A CK-only or AK-only consumer is not forced to be portable.)
//
//  2. A slot is not a byte sequence. Both families are exactly 4 bytes and both assert it,
//     so memcpy'ing slot buffers between families, persisting them, or reinterpreting them
//     as int32_t COMPILES EVERYWHERE AND IS WRONG (AK stores a plain little-endian int32_t;
//     this family stores the most-significant half first). Cross-family transfer, storage
//     and reinterpretation of the raw bytes are outside this contract. The DMA status word
//     needed no such prohibition because nothing tempts a consumer to move it; an audio
//     buffer is tempting, so the prohibition is written down.
//
//  3. Fold the conversion into the store/load the DSP already performs; do not add a
//     conversion pass. `for (i) encode(&dst[i], out[i])` is the shape the accessor
//     vocabulary invites, and it is the shape that costs: measured here, a separate pass
//     costs 3-4x a conversion folded into the DSP's own final store (see the bullet above).
//     On AK encode/decode are the identity
//     and cost nothing, which is exactly why an author there cannot see the trap -- hence it
//     is stated in the shared contract rather than only in this backend's comments.
//===========================================================
typedef struct
{
    uint16_t wire[2];   // wire[0] first on the wire (MS half), wire[1] second (LS half)
} nora_tdm_slot_t;

_Static_assert( sizeof(nora_tdm_slot_t) == 4u,
                "a TDM wire slot must be exactly 32 bits with no padding" );

// Host<->wire conversion.
//
// Each helper declares its OWN local union rather than sharing a file-scope typedef: the
// union is an optimisation detail, not part of this interface. It exists because it lets the
// compiler treat the two halves as the W-register pair it already has; the obvious
// alternative, a shift-based decode `(src->wire[0] << 16) | src->wire[1]`, compiles to
// sl/clr/ior at THREE TIMES the cost in measurements with -Os.
// Keeping it local means no caller can accidentally build a half-swapped value by hand with
// it, which is the failure mode this whole type exists to prevent.
//
// int32_t sample -> wire order. Use at the DSP's final store.
static inline void nora_tdm_slot_encode_s32( nora_tdm_slot_t *dst,
                                                  int32_t                    sample )
{
    union { uint32_t value; uint16_t half[2]; } v;   // little-endian: half[0] = LOW

    v.value      = (uint32_t)sample;
    dst->wire[0] = v.half[1];   // MS half goes out first
    dst->wire[1] = v.half[0];
}

// wire order -> int32_t sample. Use where the DSP loads its input.
static inline int32_t nora_tdm_slot_decode_s32( const nora_tdm_slot_t *src )
{
    union { uint32_t value; uint16_t half[2]; } v;   // little-endian: half[0] = LOW

    v.half[1] = src->wire[0];   // first word received is the MS half
    v.half[0] = src->wire[1];

    return (int32_t)v.value;
}

// Scale one slot by a Q15 gain (0 = silence, 0x8000 = unity) and store it back in WIRE
// order. It lives here for the same reason encode/decode do: no caller should hand-build a
// wire value manually. A gain stage is the one DSP operation this repo's audio path
// runs on every slot of every block, so the loop that does it belongs next to the layout
// knowledge it depends on.
//
// WHY Q15 AND NOT Q31. On this core a 16x16 multiply is ONE cycle and >>16 is FREE -- it is
// just naming the high word of the product pair. A Q31 gain forces `((int64_t)x * g) >> 31`,
// which XC16 compiles to a ___muldi3 call: a full 64x64 signed multiply, measured at ~88
// cycles per sample and 63% of this ISR. The multiply was never the cost; the shift WIDTH was.
//
// PRECONDITION: gain_q15 <= 0x8000 (unity). The type is uint16_t because that is the
// multiplier's natural operand, NOT because values above unity are supported -- 0xFFFF would
// be ~2.0 and can overflow the slot. Attenuation only; a caller wanting gain scales elsewhere.
//
// EXACT OVER THE AUDIO PAYLOAD, not over all 32 bits. Splitting x into a signed high half and
// an unsigned low half -- which is how it already sits in the two wire words, so nothing is
// reassembled:
//
//     x * g / 2^15  =  2 * (x_hi * g)  +  (x_lo * g) / 2^15
//
// The form below computes ((x_lo * g) >> 16) << 1 instead of (x_lo * g) >> 15, so bit 0 of
// the 32-bit slot is always 0 on the way out. The codec word is 24-bit left-justified, so
// bit 0 is not an audio bit and this is exact where it matters; on an arbitrary 32-bit value
// it is not. OR in ((lo >> 15) & 1u) for two more cycles if a caller ever needs all 32 bits.
//
// The two ends of the range need no special case, which is what keeps this loop's cost
// CONSTANT -- the property the DSP-load measurement depends on:
//   g == 0x8000  ->  y == x in every audio bit (bits 31..1 are reproduced exactly; only
//                    bit 0 is cleared). Unity is 100% of the steady state on a passthrough.
//   g == 0       ->  y == 0, a true digital zero, with no branch
//
// No overflow is possible while the precondition holds (g <= unity implies |y| <= |x|), and
// the arithmetic is unsigned throughout so there is no signed-overflow UB even though the
// result is read as signed. src == dst is safe: both words are read before either is written.
static inline void nora_tdm_slot_scale_q15( const nora_tdm_slot_t *src,
                                                 nora_tdm_slot_t       *dst,
                                                 uint16_t                         gain_q15 )
{
    const int32_t  hi = __builtin_mulsu( (int16_t)src->wire[0], gain_q15 );   // MS half: signed
    const uint32_t lo = __builtin_muluu( src->wire[1],          gain_q15 );   // LS half: unsigned
    const uint32_t y  = ((uint32_t)hi + (lo >> 16)) << 1;

    dst->wire[0] = (uint16_t)(y >> 16);   // MS half goes out first
    dst->wire[1] = (uint16_t)y;
}


// One-completed-block callback (event hook), registered PER SPI instance. The
// instance's RX-block ISR calls this for each completed block: src = the RX ping/pong
// half just captured by THIS instance; dst = the TX ping/pong half of THIS instance
// to fill; user = opaque context. One callback handles exactly one physical SPI's
// RX/TX block -- there is no dst_b / "second output": when two SPI instances are
// running, each has its own callback, and any cross-instance routing (e.g. mirroring
// one input to both outputs) is the application's job, done explicitly through a
// shared buffer between the two callbacks.
// Contract: register it (set_block_callback) BEFORE the leg is started -- inst_start() in
// SINGLE mode, start_domain()/start_all_domains() in SYSTEM mode; do NOT clear it while
// running. If no callback is registered for an instance, that instance runs no
// app/DSP path (its zeroed TX half stays silent).
// Contract: when this callback is invoked, src and dst are both non-NULL. If the core
// cannot resolve either half-buffer (reload boundary / just-stopped / first block /
// fault), it skips the block instead of calling the callback -- the callee never
// NULL-checks src/dst.
// src/dst are WIRE SLOTS, not int32_t samples -- see nora_tdm_slot_t above, and
// use the encode/decode helpers. This is a lower-level contract than "here are your
// samples", deliberately: on this part the DMA boundary IS 16-bit wire words, and hiding
// that behind an int32_t buffer is exactly how a wire-order error can stay hidden.
typedef void (*nora_spi_i2s_tdm_block_cb_t)( const nora_tdm_slot_t* src,
                                                  nora_tdm_slot_t*       dst,
                                                  void*                            user );

// Opaque per-physical-SPI instance handle. The engine exposes the two co-clocked SPI
// legs (SPI1 + optional SPI2) through the accessors below; pass the handle to
// inst_configure()/inst_start()/inst_stop()/set_block_callback()/inst_get_status() to
// drive or query that one instance. The shared board/clock port is brought up once via
// open()/close(); the app owns the multi-instance ordering.
typedef struct nora_spi_i2s_tdm_inst_s nora_spi_i2s_tdm_inst_t;

// Stream status snapshot. block_count is the number of completed audio
// blocks delivered since the last start(); one block = block_frames (NORA_TDM_BLOCK_FRAMES)
// frames per direction. load is the block-ISR load monitor (same data as
// get_load()). block_deadline_miss_count is the number of
// times the RX-block ISR fell a full block behind (HALF+DONE conflict) since start()
// -- the real-time/stream-health metric for this zero-copy engine. It is DISTINCT
// from SPI HW FIFO over/underrun flags.
// `running` is the true stream-running state -- set by start(), cleared by stop().
// It is DISTINCT from `active`: `active`
// (is_active()) is the clock/source-readiness gate (e.g. external USB audio clock
// present) that the board application uses to decide whether streaming *should*
// run, and it can read true while the stream is stopped. Read `running` for "is
// the engine actually streaming", `active` for "is the clock source ready".
typedef struct {
    bool                        active;       // is_active(): clock/source readiness (NOT running)
    bool                        running;      // is_running(): stream actually started (start..stop)
    uint32_t                    block_count;  // completed blocks since start()
    uint32_t                     block_deadline_miss_count; // HALF+DONE conflicts since start()
    // Hardware over/underrun + frame-slip observations (DISTINCT from the real-time
    // block_deadline_miss_count above). frmerr_consecutive_blocks is the connector-glitch /
    // frame bit-slip run counter the app can use as a restart trigger; each err_*_block_count
    // and rx_dma_overrun_count is a per-block observation count since start().
    uint32_t                    rx_dma_overrun_count;      // RX IRQ snapshots with DMAINTn.OVRUNIF
    uint32_t                    rx_dma_other_irq_count;    // RX IRQ snapshots with neither HALF nor DONE
    uint32_t                    rx_dma_last_status;        // raw DMAINTn from the latest RX IRQ
    uint32_t                    err_rov_block_count;       // RX blocks where SPIROV was observed
    uint32_t                    err_tur_block_count;       // RX blocks where SPITUR was observed set
    uint32_t                    err_frm_block_count;       // RX blocks where FRMERR was observed
    uint32_t                    frmerr_consecutive_blocks; // consecutive RX blocks with FRMERR (0 when clean)
    nora_spi_i2s_tdm_load_t load;         // block-ISR load/time monitor
} nora_spi_i2s_tdm_status_t;


//===========================================================
// Board/clock PORT (optional hooks). The HAL core is board-free: instead of
// calling the board adapter directly, it routes pin routing + external-clock
// concerns through this fn-pointer table, which the platform layer
// (audio_app) registers via set_port(). Every field is optional; the
// fallible hooks return bool (false => start() aborts and returns false) and take
// the resolved role so the platform can act differently for master vs slave:
//   - configure_pins(role)  : PPS/GPIO routing for the role. false => unsupported
//                             pin config (e.g. a role this board cannot drive) =>
//                             start() fails. NULL => core does no pin routing.
//   - clc_passthrough(role) : CLC bypass route (slave clock fan-out). false =>
//                             start() fails. NULL => skipped.
//   - clock_source_init(role): bring up an external (e.g. USB-audio) clock. false
//                             => start() fails. NULL => no external clock to bring up.
//   - clock_source_ready(role): external-clock readiness; drives is_active() and a
//                             SINGLE non-blocking check in start() (start() does NOT
//                             wait -- it returns false if not ready, leaving retry to
//                             the platform/app). NULL => always ready (no clock gate).
//   - consume_clock_event() : read-and-clear the ext-clock stop/resume edge; NULL
//                             => always NONE.
// With NO port registered the core behaves as a self-clocked transport with no
// readiness gate (is_active()==true, no events).
//===========================================================
typedef struct {
    bool (*configure_pins)( nora_spi_i2s_tdm_clock_role_t role );
    bool (*clc_passthrough)( nora_spi_i2s_tdm_clock_role_t role );
    bool (*clock_source_init)( nora_spi_i2s_tdm_clock_role_t role );
    bool (*clock_source_ready)( nora_spi_i2s_tdm_clock_role_t role );
    nora_spi_i2s_tdm_clock_event_t (*consume_clock_event)( void );
} nora_spi_i2s_tdm_port_t;


//===========================================================
// HAL API. The transport's entry points:
//   - set_port()         : register the board/clock port (above). Call before
//                          inst_configure()/open() -- REFUSED once open or streaming;
//                          NULL reverts to the self-clocked, no-gate default.
//   - spi1()..spi4()     : per-physical-SPI instance handles (by LITERAL SPI number;
//                          NULL when this build/device has no leg on that SPI).
//   - inst_configure()   : validate + store a config_t for one instance (no HW write).
//   - set_block_callback(): register one instance's per-block event callback.
//   - open()/close()     : shared board/clock port bring-up/teardown (once for the
//                          engine; the clock role is DERIVED from the committed
//                          block-timing reference, not passed in) -- see the
//                          per-instance lifecycle block below.
//   - inst_start()/inst_stop(): per-instance transport lifecycle, SINGLE-mode and
//                          PRIMARY-only since phase 3. A single-leg app sequences
//                          inst_configure -> open() -> inst_start() -> ... -> inst_stop()
//                          -> close(). NOTE (corrected 2026-08-09): this used to read
//                          "configure each -> open() -> start(followers) ->
//                          start(block-ref)", which is the PRE-phase-3 contract -- the
//                          inst_* family cannot start followers at all now; it addresses
//                          the primary leg only and returns ERR_CONFIG_MODE otherwise.
//                          Multi-leg belongs to configure_system()/start_domain() (SYSTEM
//                          mode) -- see CONFIGURE OWNERSHIP below. inst_start()
//                          returns true only if it actually started (false, instance
//                          stopped, if the port is not open / the clock source is no
//                          longer ready / not configured / already running / rate
//                          unsupported / DMA setup fails) and never blocks.
//   - is_active()        : clock/source readiness gate (NOT running).
//   - is_running()       : block-timing-reference running state (start..stop).
//   - get_load()/get_status()       : block-timing reference (SPI1) load / status.
//   - inst_get_load()/inst_get_status(): a specific instance's load / status.
//   (DMA interrupt vectors: default (NORA_TDM_DEFINE_DMA_VECTORS=1) the HAL owns
//    the _DMAnInterrupt vectors -- the integrator writes no ISR code. Opt-out (=0): the
//    integrator owns the IVT and calls nora_spi_i2s_tdm_inst_rx_isr() from their
//    own vector. TX is interrupt-less. Channel conflict: remap in conf.h.)
//===========================================================

// Register the board/clock port (fn-pointer hooks above). Pass NULL to clear it
// (revert to the self-clocked, no-gate default). Call before inst_configure()/open()
// (e.g. from the platform layer at init). The pointer is stored, not copied -- it
// must outlive the stream (use a static/const table).
// Returns false, PORT UNCHANGED, with ERR_ALREADY_OPEN once the port is open()'d or any
// leg is running: open() consumes these hooks (clock/pins/CLC), so swapping them afterwards
// would leave the routed hardware disagreeing with the table the HAL will call next.
extern bool nora_spi_i2s_tdm_set_port( const nora_spi_i2s_tdm_port_t* port );

// Instance handles. The instance count is a build property (which legs the core builds,
// gated by conf.h's NORA_TDM_USE_SPIn), so it is configurable but not enumerated by
// the caller. instance_count() returns how many instances this build has;
// inst(i) returns the i-th handle in table order (0 = the block-timing reference) or NULL
// if i is out of range. Together they let a caller enumerate instances
// (for i in 0 .. instance_count()-1: inst(i)) -- e.g. a future CMSIS-SAI wrapper mapping
// Driver_SAI0 -> inst(0), Driver_SAI1 -> inst(1) (that wrapper is not built yet).
// spiN() names a LITERAL PHYSICAL SPI, not a position: spi2() is the leg driving SPI2 no
// matter where it sits in the instance list, and NULL when no leg in THIS build drives
// SPI2. (It used to mean "the second instance in list order", which happened to agree
// only because the default list is SPI1-then-SPI2.) NULL for an absent instance is
// exactly what the canonical contract specifies, so it is the answer for a physical SPI
// this build does not use AND for one this DEVICE does not have.
//   - spi3() can only resolve on CK256MP508 (SPI1/2/3), and only if a conf.h instance row
//     targets SPI3. On CK64MC105 (no SPI3) nothing should target it, so it reads NULL --
//     by configuration, not by an enforced device check (see the note in the .c).
//   - spi4() is declared for contract completeness and ALWAYS returns NULL: no dsPIC33CK
//     part this HAL supports has a fourth SPI (TDM_SPI_INST_COUNT == 3).
// Use the handle with set_block_callback().
extern uint8_t                       nora_spi_i2s_tdm_instance_count( void );
extern nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_inst( uint8_t index );
extern nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi1( void );
extern nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi2( void );
extern nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi3( void );
extern nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi4( void );

//===========================================================
// CO-CLOCKED BLOCK (dual-codec, single-producer). The four entries below serve one
// logical leg's callback filling a sibling leg's TX, plus the phase probes that measure
// their alignment. A single-leg or independent-instance consumer never calls them.
//
// "A consumer here does not call it" is NOT a licence to omit: every NORA backend whose
// silicon can co-clock two legs implements all four, under these names, with these
// semantics -- that is what lets a co-clocked application move between families. Unused,
// they cost zero bytes (per-function sections + section GC; measured on this part in the
// DMA declaration's R0.1 result). This build has no caller: they are here because the
// contract requires them, and the linker is what decides whether they ship.
//
// USAGE, not stability: these are not part of the minimal single-leg transport surface,
// so a consumer that calls them is declaring a co-clocked topology, and the phase probes
// are diagnostics, not a control loop.
//===========================================================

// Return one instance's current writable TX ping-pong half (the half NOT being
// transmitted), or NULL if inst is NULL/stopped/unresolved. Lets an app produce one
// instance's output from ANOTHER instance's block callback so two co-clocked codecs
// stay sample-aligned (call at a block boundary; co-clocked siblings share the phase).
// NULL-check before writing.
extern nora_tdm_slot_t* nora_spi_i2s_tdm_inst_tx_fill_ptr( nora_spi_i2s_tdm_inst_t* inst );

// Result of inst_tx_fill_mirror() -- lets the caller distinguish a transient "position not yet
// resolvable" (reload boundary / just-started) from a genuine "target half is being transmitted"
// so it can tolerate the former (skip one block, resync only if persistent) but fault on the latter.
typedef enum {
    NORA_TDM_MIRROR_OK = 0,                   // *dst set to the safe (non-transmitting) target half
    NORA_TDM_MIRROR_UNSAFE_ACTIVE_HALF,       // target half == the half inst is transmitting NOW; *dst=NULL
    NORA_TDM_MIRROR_UNRESOLVED_DMA_POSITION,  // inst's live TX-DMA address is out of buffer range; *dst=NULL
    NORA_TDM_MIRROR_BAD_ARGUMENT,             // NULL arg / stopped inst / ref_fill_half outside ref buffer; *dst=NULL
} nora_spi_i2s_tdm_mirror_result_t;

// Mirror a reference instance's fill half onto THIS instance's TX buffer (target selected
// DETERMINISTICALLY from ref_fill_half -- valid for the whole block; a live-DMA read is used only
// as a secondary safety veto). For the co-clocked single-producer dual-codec path: pass ref = the
// producing/reference leg and ref_fill_half = the `dst` its block callback received; on OK,
// *dst = the same-index (not-transmitting, full-block-valid) half of the target `inst`. Returns a typed result:
// on OK *dst is the writable half; on UNSAFE_ACTIVE_HALF / UNRESOLVED_DMA_POSITION / BAD_ARGUMENT
// *dst is NULL and the caller must NOT write B this block (UNSAFE = fault now; UNRESOLVED = a
// transient the caller tolerates for a few blocks then resyncs). Keeps co-clocked siblings
// sample-aligned and race-free.
extern nora_spi_i2s_tdm_mirror_result_t nora_spi_i2s_tdm_inst_tx_fill_mirror(
        nora_spi_i2s_tdm_inst_t*       inst,
        const nora_spi_i2s_tdm_inst_t* ref,
        const nora_tdm_slot_t*         ref_fill_half,
        nora_tdm_slot_t**              dst );

// Phase probe: which TX ping-pong half is this instance's DMA transmitting NOW?
// 0 = ping, 1 = pong, -1 = unresolved. For measuring co-clocked sibling alignment.
extern int nora_spi_i2s_tdm_inst_tx_active_half( nora_spi_i2s_tdm_inst_t* inst );

// Phase probe (finer): TX DMA current read position as a SLOT offset into [0, 2*half),
// i.e. one unit = one sample of one wire slot -- not bytes, and not a family's DMA element.
// This backend IS such a family: its TX DMA element is a 16-bit wire word, so the raw
// address advances twice per slot and the conversion happens here, inside the HAL, exactly
// so that a diff of two legs means the same thing it means on AK. -1 if unresolved. Diff of
// two co-clocked legs = their sub-block sample offset.
extern int32_t nora_spi_i2s_tdm_inst_tx_active_pos( nora_spi_i2s_tdm_inst_t* inst );

// Register the per-completed-block callback for one SPI instance. The callback
// receives that instance's RX half just completed and the TX half it may fill.
// Register BEFORE the leg is STARTED -- inst_start() in SINGLE mode, start_domain() /
// start_all_domains() in SYSTEM mode, where inst_start() is itself refused
// (ERR_CONFIG_MODE). This call is legal in BOTH config-ownership modes and on any leg:
// a callback is per-instance state, not part of the SINGLE/SYSTEM API split.
// Returns false (and changes nothing) on a contract
// violation: NULL inst, or the instance is running and the (cb,user) pair would
// change -- the callback must not be swapped or cleared mid-stream. Re-registering the
// identical (cb,user) while running is a no-op and returns true.
extern bool nora_spi_i2s_tdm_set_block_callback( nora_spi_i2s_tdm_inst_t* inst,
                                                      nora_spi_i2s_tdm_block_cb_t cb,
                                                      void* user );

// ---- Per-instance lifecycle (the app owns multi-instance ordering) ----
// open() brings up the SHARED board/clock port (external clock + pins + CLC) ONCE for
// the engine. It takes NO role: the role handed to the port hooks is DERIVED from the
// committed block-timing reference, so the pin/clock direction cannot disagree with the
// configured stream -- which is why inst_configure() must run FIRST. Returns false (do not
// start any instance) if that leg is not configured (ERR_NOT_CONFIGURED), the clock can't be
// brought up / isn't ready, or a pin/CLC hook rejects the role. Idempotent: a second open()
// succeeds without re-running the hooks. With no port registered it is a no-op success. It
// never blocks and touches no SPI/DMA. close() is the symmetric teardown: no hardware
// teardown by design (the HAL never tears down PPS/CLC or the clock -- other peripherals may
// depend on them; reserved for a future clock-deinit hook), but it DOES clear the open state,
// so the next start needs a fresh open(). close() returns false with ERR_ALREADY_RUNNING
// while any leg still runs -- stop first.
// inst_configure/inst_start/inst_stop act on ONE instance handle (spi1()/spi2()) -- the
// PRIMARY one, in SINGLE mode only (see CONFIGURE OWNERSHIP below). A single-leg app
// sequences them: inst_configure -> open() -> inst_start -> ... -> inst_stop -> close().
// inst_start arms only that instance and REQUIRES the shared port to be open()'d
// (ERR_NOT_OPEN otherwise); it also re-checks clock readiness immediately before arming,
// so a source that dropped between open() and start is caught (ERR_CLOCK_NOT_READY).
//
// CORRECTED 2026-08-09: this paragraph used to say the app sequences "configure each ->
// open() -> start(followers) -> start(block-ref) -> ... -> stop each (block-ref first to
// halt the cadence)". That was the pre-phase-3 contract and is now false in the strong
// sense: the inst_* family REFUSES a non-primary leg (ERR_CONFIG_MODE), so no app can
// walk followers through it. Multi-leg ordering is start_domain()'s job, internally, which
// is the whole point of the arm/go split: it arms every member, then releases the
// non-master legs, then the CLOCK-MASTER leg LAST -- its BCLK/FS only starts once the
// slaves are listening. (Corrected again 2026-08-09: the first rewrite said "the
// block-timing reference last on start / first on stop", which is not what the code
// orders. Block-timing reference and clock master are SEPARATE AXES -- this HAL says so
// itself -- so a SLAVE block-timing reference in a domain with a master leg is not last.
// The stop side deliberately promises nothing: stop_domain() walks the leg table, and that
// order is an implementation detail, not a contract.) For the co-clocked SPI1+SPI2 engine,
// "followers" = SPI2 and the block-timing reference = SPI1.
// CONFIGURE OWNERSHIP (phase 3): all three inst_* calls are the SINGLE-mode, PRIMARY-only
// family. They address only the PRIMARY leg (on this board the block-timing reference, SPI1)
// and only while the committed configuration belongs to them; otherwise they return false +
// ERR_CONFIG_MODE and touch no hardware. Concretely: inst_configure() is refused once
// configure_system() has committed the stream; inst_start()/inst_stop() are refused both then
// and before the FIRST inst_configure() (nothing is committed yet, so there is no leg to start
// or stop). The mode itself is deliberately NOT queryable -- the ERR_CONFIG_MODE verdict is the
// whole public surface, so no caller can branch on the mode and re-derive the policy. close()
// does NOT reset it: a closed stream keeps its committed shape, which is what makes
// stop -> close -> open -> start a restart through the same API family.
// The gate ORDER differs between the two directions on purpose: inst_configure() answers a call
// made while open with ERR_ALREADY_OPEN (unchanged from phase 2), while inst_start() answers a
// mode violation BEFORE ERR_NOT_OPEN -- a call the caller may not make at all must not be
// answered with advice to call open().
extern bool nora_spi_i2s_tdm_open( void );
extern bool nora_spi_i2s_tdm_close( void );
extern bool nora_spi_i2s_tdm_inst_configure( nora_spi_i2s_tdm_inst_t* inst,
                                                  const nora_spi_i2s_tdm_config_t* cfg );

// Per-leg setup for the whole-system configure: the stream config PLUS the leg's sync domain,
// so BOTH are single-sourced from the caller's topology description (the sync domain is no
// longer taken only from the compile-time conf.h seed).
typedef struct {
    nora_spi_i2s_tdm_config_t stream;        // full per-leg transport config
    uint8_t                        sync_domain;   // co-clocked legs share an id; non-co-clocked legs use different ids
} nora_spi_i2s_tdm_leg_setup_t;

// Configure ALL legs in one TRANSACTIONAL call: setups[i] targets leg index i, and
// setup_count MUST equal the built leg count. Two passes with all-or-nothing semantics:
//   1. PREFLIGHT (zero side effects): every leg must be stopped, its stream must pass the
//      envelope check, its sync_domain must be < 32, each sync domain may contain at most one
//      clock MASTER, and legs sharing a sync domain must agree on the frame interpretation
//      (format/word_bits/slots/block/SPIFE/CKP/CKE/fs_shape). If ANY check fails the whole call
//      is rejected and NOT a single leg is touched. (start_domain re-checks these invariants at
//      start, so the per-leg inst_configure() path is guarded too.)
//   2. COMMIT: only after a fully clean preflight, every leg's config + sync_domain +
//      config_valid are stored together. There is thus no partially-configured state --
//      never SPI1 on the new config while SPI2 keeps the old.
// A successful call commits SYSTEM mode. Note the deliberate ASYMMETRY with the inst_* family:
// this call does NOT check the current mode, so it may full-recommit from NONE, SINGLE or
// SYSTEM -- it is the one way back out of SINGLE ownership. It is still gated on the port being
// CLOSED (ERR_ALREADY_OPEN) and on every leg being stopped, which is what keeps the recommit
// safe; adding a mode gate here would strand a SINGLE-committed stream forever.
// The caller owns the stop->configure->start contract (configure_system does NOT stop a
// running transport; it rejects one). Replaces per-leg inst_configure + any app-side role
// rewrite: the caller hands resolved per-leg setups and gets all-or-nothing.
extern bool nora_spi_i2s_tdm_configure_system( const nora_spi_i2s_tdm_leg_setup_t* setups,
                                                    uint8_t setup_count );

// Read one leg's COMMITTED setup (the config stored by inst_configure/configure_system, plus
// its sync domain)
// into *setup. Returns false -- and touches neither *setup nor the last-error (pure query) --
// if inst is NULL, setup is NULL, or the leg is not configured (config_valid == false). Lets a
// board port hook route that leg's pins/CLC from the committed clock role with no side table,
// and lets a caller distinguish "unconfigured" from a valid SLAVE (role value 0). An optional
// leg left unconfigured returns false, so the caller can SKIP it rather than assume a role.
// Deliberately silent on the error channel: a port hook may call this from inside open(), and
// a query must not overwrite the real failure code the caller is about to read.
extern bool nora_spi_i2s_tdm_inst_get_setup( const nora_spi_i2s_tdm_inst_t* inst,
                                                  nora_spi_i2s_tdm_leg_setup_t* setup );

extern bool nora_spi_i2s_tdm_inst_start( nora_spi_i2s_tdm_inst_t* inst );
extern bool nora_spi_i2s_tdm_inst_stop( nora_spi_i2s_tdm_inst_t* inst );
// NOTE: the internal arm/go split (program+arm DMA/SPI with the module OFF, then release SPIEN
// back-to-back so co-clocked legs latch one FS edge = phase-locked) is NOT public. It has no
// armed-state / open-gate of its own, so exposing it would let a caller enable SPI out of
// sequence. Phase-locked co-clocked startup is delivered through start_domain() (which arms
// then releases internally); a single leg uses inst_start().

// Sync-domain group start/stop -- the SYSTEM-mode API (stream committed via configure_system()).
// A domain = the set of legs sharing sync_domain. start_domain arms all members then releases
// SPIEN back-to-back (non-master legs first, clock-master last) so co-clocked members latch one
// FS edge = phase-locked. open() must run first. Both return false with ERR_CONFIG_MODE if the
// stream was committed via inst_configure() (SINGLE mode) -- a SINGLE-mode stream starts/stops
// through inst_start()/inst_stop().
// start_domain is NON-DESTRUCTIVE on rejection: an already fully-running domain is idempotent
// success, and a PARTIALLY running one is refused with ERR_ALREADY_RUNNING and left exactly as
// it was, so a spurious re-assert can never cut live audio. An unconfigured member is
// ERR_NOT_CONFIGURED, more than one clock master or a framing disagreement is ERR_TOPOLOGY, and
// a domain with no member at all is ERR_BAD_INSTANCE -- an unknown domain id is an error rather
// than a silent success, in both directions. stop_domain returns true after the (idempotent)
// teardown of an existing domain.
//
// *** UNVERIFIED ON CK SILICON, AND IT WILL STAY THAT WAY. Read this before relying on it. ***
// The words "phase-locked" and "latch one FS edge" above describe the INTENT of the register
// order. They have never been observed on a scope on a dsPIC33CK, and they cannot be: showing
// zero-frame skew needs two co-clocked legs, EV88G73A carries CODEC-A only and is not designed
// to drive a second codec, and no dual-codec CK board exists. That test was CANCELLED on
// 2026-08-09 as permanently unavailable -- not deferred. The AK boards do have two legs, but a
// measurement there says nothing about this silicon.
// Concretely, on CK today:
//   - the single-leg path (inst_start) IS hardware-proven, in both clock roles;
//   - the arm/go split itself IS exercised -- the master FS/CLC engage branch ran on hardware;
//   - the multi-leg meanings of ERR_TOPOLOGY (>1 clock master in a domain, framing disagreement
//     between members) have NEVER executed -- also cancelled, same missing board;
//   - so a 2-leg domain on CK is code that has been reasoned about and never run.
// If you are the first person to wire two CK legs: treat every claim in this comment block as a
// hypothesis to test, not as a guarantee to build on.
extern bool nora_spi_i2s_tdm_start_domain( uint8_t domain );
extern bool nora_spi_i2s_tdm_stop_domain( uint8_t domain );

// Whole-system start/stop: every sync domain present in the leg table, each exactly once, so a
// caller never enumerates individual logical legs. There is no cross-domain start-ordering
// constraint here (NOTE: that is NOT full independence -- source readiness is engine-wide and
// shared resources such as the FS_50PCT CLC and the board clock port are not per-domain).
// start_all_domains runs TWO passes so a later domain's failure can never tear down a domain that
// was already running before the call, nor one the call never touched: pass 1 classifies every
// distinct domain side-effect-free and rejects the WHOLE call touching NOTHING if any is partially
// running or invalid; pass 2 starts only the fully-stopped ones and, on failure, rolls back only
// the domains this call actually started. ERR_NOT_CONFIGURED if no domain is configured at all.
// Both are SYSTEM-mode only (ERR_CONFIG_MODE otherwise); stop_all_domains is idempotent success.
extern bool nora_spi_i2s_tdm_start_all_domains( void );
extern bool nora_spi_i2s_tdm_stop_all_domains( void );

// Return the clock/source readiness gate. This can be true while the transport is
// stopped; use is_running() when the question is "is audio streaming now?"
extern bool nora_spi_i2s_tdm_is_active( void );

// Return true only after start() succeeds and before stop() begins.
extern bool nora_spi_i2s_tdm_is_running( void );   // true stream-running state (start..stop)

// Consume one external-clock stop/resume edge from the board port, or NONE when
// no event/hook exists.
extern nora_spi_i2s_tdm_clock_event_t nora_spi_i2s_tdm_consume_clock_event( void );  // external-clock stop/resume edge

// NOTE: the transport is RATE-AGNOSTIC -- there is intentionally NO sample-rate API
// (no notify / get / set-callback / is-supported / rate_state). The HAL runs at the
// configured BRG (master) or the incoming external clock (slave) and never derives
// anything from a sample-rate value. Sample-rate POLICY is NOT a HAL property: the
// product/board's supported-rate set lives in the app layer (APP_SAMPLE_RATE_IS_SUPPORTED),
// used by the CMSIS-SAI wrapper to validate ARM_SAI AUDIO_FREQ. Runtime rate DETECTION +
// the stop->reconfigure->start it drives live in the application.

// Snapshot the load monitor / status. The singleton forms report the block-timing
// reference (SPI1); the inst forms report a specific instance (use spi1()/spi2()).
// For the inst forms, block_count/deadline_miss/load AND running are that instance's;
// only active (the clock/source readiness gate) is engine-wide/shared.
// clear_peak resets that instance's min/max/event peaks after the snapshot.
extern bool nora_spi_i2s_tdm_get_load( nora_spi_i2s_tdm_load_t* monitor, bool clear_peak );
extern bool nora_spi_i2s_tdm_get_status( nora_spi_i2s_tdm_status_t* status, bool clear_peak );
extern bool nora_spi_i2s_tdm_inst_get_load( nora_spi_i2s_tdm_inst_t* inst,
                                                 nora_spi_i2s_tdm_load_t* monitor,
                                                 bool clear_peak );
extern bool nora_spi_i2s_tdm_inst_get_status( nora_spi_i2s_tdm_inst_t* inst,
                                                   nora_spi_i2s_tdm_status_t* status,
                                                   bool clear_peak );

// Engine-wide TDM-active COMBINED-occupancy ("TDMsum") profiler control/readout.
//
// These operate on a single engine-wide profiler shared by every TDM RX-block ISR (see
// nora_spi_i2s_tdm_tdmsum_t). All three bracket the access with EVERY configured leg's
// RX-DMA IE mask so the read/reconfigure is consistent against the (mutually non-preempting)
// TDM ISRs on this 16-bit core -- the caller does NOT mask.
//
// _tdmsum_configure() sets the common window length (in raw high-res-timer counts, e.g. the
// block deadline converted to counts) and re-bases the window grid, clearing depth/peaks. Call
// it once the deadline is known and again whenever it changes (rate change / new stream epoch).
// _tdmsum_reset() re-bases the grid and clears depth/peaks but KEEPS the window length (use on
// stop/resume). _tdmsum_get() snapshots the peak/saturation, clearing them when clear_peak.
//
// Declared only when NORA_TDM_SUMPROF is 1 (the default). With 0 the profiler, its ISR hooks
// and these three entry points are not compiled -- a reference then fails at compile time
// rather than silently returning a never-updated zero snapshot.
#if NORA_TDM_SUMPROF
extern void nora_spi_i2s_tdm_tdmsum_configure( uint32_t window_period_ticks );
extern void nora_spi_i2s_tdm_tdmsum_reset( void );
extern bool nora_spi_i2s_tdm_tdmsum_get( nora_spi_i2s_tdm_tdmsum_t* out,
                                              bool clear_peak );
#endif


// Last-error diagnostic. The bool-returning calls (set_port / open / close / inst_configure /
// inst_start / inst_stop / set_block_callback) collapse several failure causes into one
// `false`; get_last_error()
// returns the most
// specific reason recorded by the most recent such call (ERR_NONE after a success). This
// is a DEBUG aid only -- NOT stream health: deadline misses / block counts live in
// get_status(), not here. It is the "last failed API reason", not a per-instance latch,
// and is intentionally not strictly interrupt/multi-core safe (a plain last-writer-wins
// store -- adequate as a 16-bit-MCU debug hint).
typedef enum {
    NORA_SPI_I2S_TDM_ERR_NONE = 0,
    NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE,        // NULL / out-of-range instance handle
    NORA_SPI_I2S_TDM_ERR_BAD_ARGUMENT,        // NULL cfg / other bad argument
    NORA_SPI_I2S_TDM_ERR_NOT_CONFIGURED,      // start before a successful configure
    NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING,     // start/configure while running, or close while running
    NORA_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG,  // configure envelope rejected (format/slots/blk)
    NORA_SPI_I2S_TDM_ERR_TOPOLOGY,            // leg-table/topology validation failed,
                                                   // or a leg's sync_domain is outside 0..31.
                                                   // Phase 4 added the sync-domain meaning: a
                                                   // domain with more than one clock MASTER, or
                                                   // whose members disagree on framing (format,
                                                   // word_bits, slots_per_fs, block_frames, or
                                                   // any of the 4 edge/shape flags) -- members of
                                                   // one domain ride ONE BCLK/FS, so disagreeing
                                                   // framing is unsatisfiable, not a preference.
                                                   // UNEXERCISED: the two multi-leg meanings
                                                   // (>1 master, framing disagreement) have never
                                                   // been reached on CK hardware and never will
                                                   // be -- they need >=2 co-clocked legs and no
                                                   // dual-codec CK board exists (CANCELLED
                                                   // 2026-08-09, plan doc sec.15 D-3). The
                                                   // sync_domain range check IS covered.
    NORA_SPI_I2S_TDM_ERR_CLOCK_INIT,          // port clock_source_init hook failed
    NORA_SPI_I2S_TDM_ERR_CLOCK_NOT_READY,     // port clock_source_ready hook not ready
    NORA_SPI_I2S_TDM_ERR_PIN_CONFIG,          // port configure_pins hook failed
    NORA_SPI_I2S_TDM_ERR_CLC,                 // port clc_passthrough hook failed
    NORA_SPI_I2S_TDM_ERR_DMA_CONFIG,          // DMA channel setup failed
    // The first two became REACHABLE in phase 2 (the open-state machine); the third in phase 3
    // (configure ownership). CONFIG_MODE is returned by inst_configure() (SYSTEM-committed
    // stream, or a non-primary leg) and by inst_start()/inst_stop() (mode not SINGLE -- which
    // includes "never configured" -- or a non-primary leg). Phase 4 made the domain-API half
    // reachable too: the four *_domain(s) calls return it unless the stream was committed through
    // configure_system(), so neither ownership mode can be driven through the other's API.
    NORA_SPI_I2S_TDM_ERR_NOT_OPEN,            // start/arm attempted before a successful open()
    NORA_SPI_I2S_TDM_ERR_ALREADY_OPEN,        // configure/set_port attempted while open()'d
    NORA_SPI_I2S_TDM_ERR_CONFIG_MODE,         // wrong configure-ownership mode for this call
                                                   // (e.g. inst_* under SYSTEM / a non-primary leg,
                                                   // or start_domain under SINGLE)
} nora_spi_i2s_tdm_error_t;

extern nora_spi_i2s_tdm_error_t nora_spi_i2s_tdm_get_last_error( void );

// DMA interrupt vectors: by default (NORA_TDM_DEFINE_DMA_VECTORS=1, conf.h) the HAL
// DEFINES the _DMAnInterrupt vectors itself (one explicit RX vector per built leg),
// so the integrator writes NO interrupt/DMA code -- just registers a per-instance block
// callback. TX is interrupt-less (fire-and-forget ping-pong with auto-reload; hw.c
// enables the CPU IRQ on the RX channel only). RX/TX channel numbers come from conf.h and
// are baked in as compile-time constants so the DMA register access folds. To yield an
// IVT slot to another subsystem, either remap the SPI's channel in conf.h, or take full
// vector ownership: set NORA_TDM_DEFINE_DMA_VECTORS=0 (the HAL then defines no
// vectors) and call inst_rx_isr() below from your own _DMA<rx>Interrupt for each instance.

// RX-block ISR entry for one instance, for the NORA_TDM_DEFINE_DMA_VECTORS=0
// (vector-ownership opt-out) path: call it from your own _DMA<rx>Interrupt for that
// instance's RX channel (TX is interrupt-less -- never call it for a TX channel). It runs
// the same block work as the HAL's generated vector. A NULL inst is ignored. In the
// default turnkey build (=1) you do not call this -- the HAL's own vectors do the work.
extern void nora_spi_i2s_tdm_inst_rx_isr( nora_spi_i2s_tdm_inst_t* inst );



#endif // NORA_SPI_I2S_TDM_H
