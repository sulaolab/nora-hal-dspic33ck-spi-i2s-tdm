
//===========================================================
// INCLUDES
//===========================================================
#include "nora_spi_i2s_tdm_conf.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// The silicon layer (SPIxCON1/BRG register writer, the s_spi_dev[] device-facts table,
// per-SPI DMA-channel config) lives in nora_spi_i2s_tdm_hw.*, and the block
// counters + ISR load/time monitor in nora_spi_i2s_tdm_diag.*. So the transport
// core no longer pulls the SPI register masks (reg.h), high-res timer, debug GPIO, or
// <stdio.h> -- it orchestrates instances and runs the block ISR, delegating register
// pokes to hw.* and diagnostics to diag.*.
#include "nora_dma.h"
// The RX-block ISR body is a measured hot path, so it uses the `_hot` inlines. Including
// this header is that declaration; the non-ISR paths in this file (the public TX-fill
// pointer query, the RX-IE guard around start/stop) deliberately keep calling the
// out-of-line twins in nora_dma.h.
#include "nora_dma_dspic33ck_fast.h"
#include "nora_spi_i2s_tdm.h"
#include "nora_spi_i2s_tdm_dspic33ck_hw.h"      // silicon layer: tdm_spi_inst_t + register/DMA ops
#include "nora_spi_i2s_tdm_dspic33ck_fs_clc.h"  // CLC1 50%-FS generator (TDM master + FS_50PCT)
#include "nora_spi_i2s_tdm_diag.h"    // block counters + ISR load/time monitor (separated)
#include "nora_spi_i2s_tdm_dspic33ck_diag_fast.h" // the same diag's per-block ISR `_hot` hooks
#if NORA_TDM_SUMPROF
// The TDMsum hooks need the raw counter directly (the per-leg monitor reaches it through
// diag.c). Only the profiler needs it here, so the include follows the same switch.
#include "nora_high_res_timer.h"
#endif




//===========================================================
// Definition
//===========================================================
// Unknown-device guard is centralized in nora_spi_i2s_tdm.h: the
// NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE derivation #error's on any unsupported device.


// DMA global address-window values are owned by the DMA HAL
// (NORA_DMA_ADDR_LIMIT_LOW / _HIGH in nora_dma_dspic33ck.c), set by
// nora_dma_global_init(). SPI-TDM keeps only per-channel cfg values.




// Debug switches live with their code now: the DMA-config-error printf in the silicon
// layer (nora_spi_i2s_tdm_dspic33ck_hw.c) and the scope-GPIO/load monitor in the diagnostics
// module (nora_spi_i2s_tdm_dspic33ck_diag.c) each carry their own ENA_TDM_DBG. The transport
// core compiles with no printf/GPIO dependency.

// Per-instance DMA channel assignment comes from conf.h (NORA_TDM_SPIn_*_DMA);
// the core no longer hardcodes DMA channel index constants.

#define TDM_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define TDM_COMPILEASSERT(exp) extern int tdm_compile_assert[(exp) ? 1 : -1] __attribute__((unused))




//--------------------------------------------







//===========================================================
// Enum & Struct typedef
//===========================================================

// tdm_spi_inst_t (physical SPI instances) + the silicon device-facts table live in the
// silicon layer (nora_spi_i2s_tdm_hw.*). The transport core indexes its own leg
// table with the leg-index enum below and stores the physical SPI in each leg's spi_inst.
//
// Explicit dense leg-index enum: one TDM_SPI_LEG_<name> per leg the build carries, in
// table order, terminated by TDM_SPI_LEG_COUNT (= the built-in leg count). Leg 0 is the
// block-timing reference. Used only inside the core (leg table / inst() / spi1()/spi2()),
// so it lives here rather than in a shared header.
//
// This enum, the ping-pong buffers, the s_spi_legs[] table and the RX interrupt vectors
// were formerly GENERATED from a NORA_TDM_INSTANCE_LIST X-macro in conf.h. They are
// now written out directly, keyed off the per-leg conf.h knobs (NORA_TDM_USE_SPI2,
// NORA_TDM_SPIn_RX/TX_DMA, NORA_TDM_SPIn_SYNC_DOMAIN) and the stream geometry
// (NORA_TDM_SLOTS_PER_FS / _BLOCK_FRAMES), so conf.h no longer owns the topology --
// it supplies per-leg values and the core owns the shape. Adding a leg means adding its
// four blocks here plus its channel/domain macros in conf.h.
typedef enum {
    TDM_SPI_LEG_SPI1 = 0,
#if NORA_TDM_USE_SPI2
    TDM_SPI_LEG_SPI2,
#endif
    TDM_SPI_LEG_COUNT
} tdm_spi_leg_index_t;

// TDM_SPI_LEG_BLOCK_REF is the block-timing REFERENCE leg (leg 0) -- NOT a clock master.
// It owns the RX-block ISR that defines the block boundary and is what the singleton
// is_running()/get_status()/get_load() report. The clock role (master/slave) is a
// separate, per-instance concern carried in nora_spi_i2s_tdm_config_t.clock_role. The
// current co-clocked topology requires exactly one BLOCK_REF, and it must be leg 0.
#define TDM_SPI_LEG_BLOCK_REF   0   // first leg is the block-timing reference

// Private SPI leg descriptor == the public opaque instance handle
// (nora_spi_i2s_tdm_inst_t). One physical SPI leg per row owns the SPI
// peripheral, RX/TX DMA channels, ping-pong buffers, its own block callback, and
// its own diagnostics. Each instance's RX-block ISR delivers ONLY this instance's
// RX/TX block to this instance's callback.
struct nora_spi_i2s_tdm_inst_s {
    tdm_spi_inst_t spi_inst;
    nora_dma_channel_t rx_dma_ch;
    nora_dma_channel_t tx_dma_ch;
    /*
     * Cached before DMA/SPI are enabled by inst_start().  The generic silicon
     * table maps spi_inst -> SPIxSTATL, but doing that table lookup for every
     * completed block made framed-error sampling an out-of-line ISR call.  A
     * valid leg has one immutable physical SPI, so cache its status-register
     * address once and let the ISR perform only the required volatile read and
     * W0C-safe acknowledgement.  Never expose this pointer outside the HAL.
     */
    volatile uint16_t *spi_statl;
    // Wire slots, NOT int32_t samples: the DMA boundary is 16-bit wire words on this part.
    // See nora_tdm_slot_t for the wire-order representation.
    nora_tdm_slot_t *rx_buffer;
    nora_tdm_slot_t *tx_buffer;
    uint32_t       buffer_slot_count;   // total SLOTS (2 * slots * blk) -- both ping+pong
    uint8_t        geom_slots_per_fs;   // THIS leg's compile-time slots/FS (buffer geometry)
    uint16_t       geom_block_frames;   // THIS leg's compile-time frames per ping/pong half
    nora_spi_i2s_tdm_config_t config;         // includes this leg's OWN clock role
    bool           config_valid;
    // Block-timing / singleton-reporting REFERENCE leg (leg 0). Each leg
    // already times itself via its own RX-block ISR (P1/P2), so this is NOT a clock-role
    // flag (clock role is per-leg in config, P3 Stage 1) and NOT a hard timing coupling --
    // it only marks which leg is_running()/get_status()/is_active() report and which RX
    // block the demo treats as primary. (When per-instance clocks land -- P3 Stage 2 --
    // the "exactly one, leg 0" topology rule generalizes; today the stream is co-clocked.)
    bool           is_block_timing_master;
    /*
     * Co-clocked grouping: legs that share one BCLK/FS carry the SAME id. A THIRD axis,
     * orthogonal to both neighbours above -- is_block_timing_master says who defines the
     * block boundary, config.clock_role says who DRIVES the clock, and this says who rides
     * the SAME clock. A domain holds at most one CLOCK_MASTER; two legs on independent
     * clocks are different domains even if both are masters.
     *
     * Seeded from conf.h at build time and overwritten by configure_system() (phase 4),
     * which is what makes the caller's topology table the runtime source of truth. Range
     * 0..31 -- start_all_domains() dedups and rolls back through a 32-bit domain mask, and
     * that limit is enforced at both entry points as well as by a compile assert per row.
     */
    uint8_t        sync_domain;
    nora_spi_i2s_tdm_block_cb_t block_cb;     // this instance's block callback
    void                            *block_user;   // opaque user pointer for block_cb
    nora_spi_i2s_tdm_diag_t     diag;         // this instance's counters + ISR load monitor
    volatile bool                    running;      // this instance started (inst_start..inst_stop)
};
typedef struct nora_spi_i2s_tdm_inst_s tdm_spi_leg_t;

typedef struct
{
    tdm_spi_leg_t                      *legs;
    uint8_t                             leg_count;
    const nora_spi_i2s_tdm_port_t *port;

    /*
     * ENGINE-WIDE open state: set by a successful open(), cleared by close(). It is a
     * property of the SHARED port (external clock + pins + CLC), which is why it lives
     * here and not per leg -- open() brings that up once for every leg.
     *
     * It is what turns four bool return types into load-bearing gates (phase 2): the port
     * may not be swapped while open (set_port), a config may not be committed while open
     * (inst_configure -- open() derives the clock role FROM that config), a leg may not be
     * armed while closed (inst_start -> ERR_NOT_OPEN), and close() may not run under a live
     * stream. Deliberately separate from each leg's `running`: open means "the shared port
     * is routed and its clock was ready", not "audio is moving".
     */
    bool                                opened;

    // Block callback, diagnostics, and the running flag are now owned PER INSTANCE
    // (in tdm_spi_leg_t), not by the stream, so SPI1 and SPI2 each deliver their own
    // block, measure their own ISR load, and start/stop independently.
} tdm_stream_t;


//===========================================================
// Function Prototype
//===========================================================

// pin routing + CLC pass-through live on the board adapter and are reached
// through the registered port callbacks, NOT called by name -- the core no
// longer includes audio_app_board.h.

// Silicon-layer SPI register/DMA ops (tdm_spi_apply_config, *_dma_trigger_enable,
// *_module_enable, *_irq_*, *_soft_stop, *_dma_config) now live in the hw module and
// are reached as nora_spi_i2s_tdm_hw_*() taking a tdm_spi_inst_t + raw fields.

// Per-instance teardown/clear helpers (one physical SPI's own DMA buffers/channels).
static void        tdm_inst_clear_buffers( const tdm_spi_leg_t *leg );
static void        tdm_inst_soft_stop_dma( const tdm_spi_leg_t *leg );
static void        tdm_inst_stop_impl( tdm_spi_leg_t *leg );
// Start, split at the SPIEN boundary (phase 4). Both private: publishing go() would let a
// caller enter SPIEN in an order that has never been shown to bring the FS/CLC master up.
static bool        tdm_inst_arm( tdm_spi_leg_t *leg );
static void        tdm_inst_go( tdm_spi_leg_t *leg );
// Sync-domain helpers (phase 4). All private: the domain state is derived from the leg table
// on demand, never stored, so no getter can go stale.
static bool        tdm_domain_framing_matches( const nora_spi_i2s_tdm_config_t *a,
                                               const nora_spi_i2s_tdm_config_t *b );
static void        tdm_stop_domain_impl( uint8_t domain );
static void        tdm_stop_all_domains_impl( void );
static void        tdm_inst_clear_dma_flags( const tdm_spi_leg_t *leg );
static void        tdm_zero_memory(void *ptr, size_t bytes);
static bool        tdm_spi_leg_is_valid( const tdm_spi_leg_t *leg );
static bool        tdm_stream_topology_is_valid( const tdm_stream_t *stream );
static const tdm_spi_leg_t *tdm_stream_get_block_timing_master_leg( const tdm_stream_t *stream );
static bool        tdm_spi_leg_get_effective_config( const tdm_spi_leg_t *leg,
                                                     nora_spi_i2s_tdm_config_t *effective_cfg );
// Open/running state machine (phase 2). Both are needed by set_port(), the file's first
// global function, so they are declared here with the rest.
static bool        tdm_any_leg_running( void );
// Configure-ownership mode (phase 3): is this leg the one the per-leg SINGLE API may touch?
static bool        tdm_inst_is_primary( const tdm_spi_leg_t *leg );
static bool        tdm_stream_ready_for_start( void );

// RX-DMA IE bracket, config-envelope validation, and the per-instance RX-block ISR
// body. Definitions live in the Local Function section; forward-declared here so the
// global readers (get_load/get_status) and the ISR wrappers above their definitions
// resolve them. tdm_rx_ie_*/tdm_rx_block are static inline (folded in the hot path).
static inline bool tdm_rx_ie_disable( nora_dma_channel_t rx_dma_ch );
static inline void tdm_rx_ie_restore( nora_dma_channel_t rx_dma_ch, bool was_enabled );
static bool        tdm_config_is_supported( const tdm_spi_leg_t* leg, const nora_spi_i2s_tdm_config_t* cfg );
// The last-error latch sits with the other diagnostics further down, but set_port() -- the
// first global function in the file -- now records ERR_NONE on success like every other
// bool-returning entry point, so the setter is forward-declared here.
static inline void tdm_set_error( nora_spi_i2s_tdm_error_t err );
// `always_inline` is intentional.  Under the project's -Os build, plain
// `static inline` was outlined as a 318-byte helper and the DMA vector called
// it.  That made rx_ch/tx_ch runtime parameters, defeating the DMA HAL's
// compile-time channel folding.  The generated vector owns one fixed channel
// and one fixed block geometry, so duplicating this body is the correct
// code-size-for-deadline trade here.
static inline __attribute__((always_inline)) void
tdm_rx_block( tdm_spi_leg_t* inst, nora_dma_channel_t rx_ch, nora_dma_channel_t tx_ch, uint32_t half_pos );




static inline void  tdm_get_src_ptr( nora_dma_status_t dma_stat,
                                       const nora_tdm_slot_t*  const pRxDat,
                                       uint32_t        half_pos,
                                       const nora_tdm_slot_t** src_pptr );
static inline void  tdm_get_dest_ptr( uint32_t dma_tx_addr, nora_tdm_slot_t* const pTxDat, uint32_t half_pos, nora_tdm_slot_t** dest_pptr );
// local_copy_to_CODEC / local_drc_df2t_path / local_filter_cascade_chm were
// moved to audio_app.c. The latter two are declared in audio_app.h.





//===========================================================
// Variables
//===========================================================

#define DMA_BUF_PING_POS     (0)   // ping half is always at offset 0; pong half = slots*blk per leg

// Per-instance ping/pong half size (words) = slots * blk for THIS row. Both the buffer
// size (2 * half) and the ISR's pong-half offset derive from it; passed as a compile-
// time literal into the generated ISR bodies so the hot-path pointer math stays folded.
#define TDM_LEG_HALF_SLOTS(slots, blk)   ((slots) * (blk))

// Per-instance RX/TX ping-pong buffers, one explicit Rx_<name>/Tx_<name> pair per leg.
// Each is 2 * slots * blk words (Ping + Pong) using THIS leg's slots/blk, so a leg may
// carry its own geometry (both default legs take the stream-wide macros). The names
// follow the leg names so the leg table can wire them per leg.
static nora_tdm_slot_t    Tx_SPI1[ 2 * TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */
static nora_tdm_slot_t    Rx_SPI1[ 2 * TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */
#if NORA_TDM_USE_SPI2
static nora_tdm_slot_t    Tx_SPI2[ 2 * TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */
static nora_tdm_slot_t    Rx_SPI2[ 2 * TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */
#endif

// the application DSP float work buffers (f_A_Data, f_A_Data_chm,
// f_B_Data_chm) live in the demo app (audio_app.c), which owns AND
// clears them (before start()). The HAL core only owns/clears its DMA ping-pong
// buffers (the Rx_<name>/Tx_<name> pair per instance).




// RB15 / USB audio clock state and ISR moved to audio_app_board.c.


//===========================================================
// DMA channel + SPI allocation
//
// The DEVICE FACTS table (s_spi_dev[]: SPIxBUF/CON1/BRG/IMSK + DMA trigger CHSELs +
// CPU IRQ bits, indexed by tdm_spi_inst_t) lives in the silicon layer
// (nora_spi_i2s_tdm_hw.*). Here the transport core keeps only its DRIVER
// ALLOCATION: s_spi_legs[], one physical SPI leg per row with the SPI instance, RX/TX
// DMA channels, owned ping-pong buffers, and current logical config. start() passes
// each leg's spi_inst + DMA channels/buffers down to the hw_* ops.
//===========================================================

// One explicit row per leg: physical SPI + RX/TX DMA channels + its own Rx_<name>/Tx_<name>
// ping-pong buffers. Leg 0 gets is_block_timing_master=1 (the singleton-reporting block
// reference); every other leg gets 0. Each leg's CLOCK role is NOT forced here -- it comes
// from the leg's own config (set by the integrator per leg: a follower is configured SLAVE
// because it rides the shared clock, not because the HAL forces it). The sync_domain value
// here is only the build-time SEED; configure_system() overwrites it from the caller's
// topology table, which is the runtime source of truth (see conf.h).
static tdm_spi_leg_t s_spi_legs[] =
{
    [TDM_SPI_LEG_SPI1] =
    {
        .spi_inst               = TDM_SPI1,
        .rx_dma_ch              = (NORA_TDM_SPI1_RX_DMA),
        .tx_dma_ch              = (NORA_TDM_SPI1_TX_DMA),
        .rx_buffer              = Rx_SPI1,
        .tx_buffer              = Tx_SPI1,
        .buffer_slot_count      = TDM_ARRAY_SIZE(Rx_SPI1),
        .geom_slots_per_fs      = (uint8_t)(NORA_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(NORA_TDM_BLOCK_FRAMES),
        .is_block_timing_master = 1,   // leg 0 = block-timing reference (NOT the clock role)
        .sync_domain            = (uint8_t)(NORA_TDM_SPI1_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },  /* rest zero; start() calls diag_reset() */
    },
#if NORA_TDM_USE_SPI2
    [TDM_SPI_LEG_SPI2] =
    {
        .spi_inst               = TDM_SPI2,
        .rx_dma_ch              = (NORA_TDM_SPI2_RX_DMA),
        .tx_dma_ch              = (NORA_TDM_SPI2_TX_DMA),
        .rx_buffer              = Rx_SPI2,
        .tx_buffer              = Tx_SPI2,
        .buffer_slot_count      = TDM_ARRAY_SIZE(Rx_SPI2),
        .geom_slots_per_fs      = (uint8_t)(NORA_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(NORA_TDM_BLOCK_FRAMES),
        .is_block_timing_master = 0,   // follower: rides leg 0's block boundary
        .sync_domain            = (uint8_t)(NORA_TDM_SPI2_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },  /* rest zero; start() calls diag_reset() */
    },
#endif
};

static tdm_stream_t s_stream =
{
    .legs           = s_spi_legs,
    .leg_count      = (uint8_t)TDM_ARRAY_SIZE(s_spi_legs),
    .port           = NULL,
    .opened         = false,
};

// The registered board/clock port is owned by the singleton stream context.
// The table is held by pointer, not copied, so the caller must provide a
// static/long-lived object. NULL is the board-free default: no pin routing,
// no external clock gate, and no clock-change events.

// Block-completion callback ownership lives PER INSTANCE (tdm_spi_leg_t.block_cb).
// Each instance's RX-block ISR invokes its own callback for each completed block;
// if NULL, that instance runs no app/DSP path and start()'s zeroed TX half stays
// silent until a callback fills it. Register before start(), do not clear while
// running; set_block_callback() updates the pair under that instance's RX DMA IE mask.

// Stream-health counters and the ISR load/time monitor live in the separated
// diagnostics module (nora_spi_i2s_tdm_diag.*); each instance (tdm_spi_leg_t)
// holds its own nora_spi_i2s_tdm_diag_t and updates it ONLY through the diag_*
// functions. Each instance's RX-block ISR feeds its diag (note_block / check_deadline
// / isr_begin / isr_end); start() resets every instance's diag. The singleton
// get_load()/get_status() report the block-timing-reference instance's diag under its
// RX DMA IE mask because 32-bit reads are non-atomic on this 16-bit core. The
// deadline metric is software/real-time, not SPIROV/SPITUR hardware status.

// Lifecycle running state is owned PER INSTANCE (tdm_spi_leg_t.running): set by a
// successful inst_start() and cleared by inst_stop(), exposed via
// inst_get_status().running and (for the block-timing reference) is_running(). This is
// deliberately separate from is_active(), which reports clock/source readiness.

// The leg table must have at least the BLOCK_REF row and not exceed the silicon SPI count.
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) >= 1u );
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) <= (size_t)TDM_SPI_INST_COUNT );

// Per-instance geometry sanity (compile-time), one set per leg: slots/blk fit their leg
// fields (uint8_t / uint16_t), the 2*slots*blk word count cannot overflow int32 indexing,
// and the leg's buffer is exactly that size. The default legs use stream-wide macros
// already range-checked in conf.h; these asserts also cover a leg given its own slots/blk,
// so the per-instance geometry promise has teeth. Kept as a macro over (name, slots, blk)
// so the five checks are stated once and each leg is one line -- this is a repetition of
// identical checks, not the retired topology list: it declares no leg and no channel.
#define TDM_LEG_GEOM_ASSERTS(name, slots, blk, dom)                                       \
    TDM_COMPILEASSERT( (slots) > 0 && (slots) <= 255 );                                   \
    TDM_COMPILEASSERT( (blk)   > 0 && (blk)   <= 65535 );                                 \
    TDM_COMPILEASSERT( (slots) <= (2147483647 / (2 * (blk))) );                           \
    TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_##name) == (2u * (slots) * (blk)) );              \
    TDM_COMPILEASSERT( (dom)   >= 0 && (dom)   < 32 )  /* 32-bit domain mask in start_all_domains() */

TDM_LEG_GEOM_ASSERTS( SPI1, NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES,
                      NORA_TDM_SPI1_SYNC_DOMAIN );
#if NORA_TDM_USE_SPI2
TDM_LEG_GEOM_ASSERTS( SPI2, NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES,
                      NORA_TDM_SPI2_SYNC_DOMAIN );
#endif


/*
 * Which API family owns the committed configuration.
 *
 * Set by whichever configure entry committed last and NEVER cleared by close() (a closed
 * stream keeps its committed shape, so the next open()->start uses the same API family --
 * that is what makes the console's stop -> close -> open -> start restart work).
 *   NONE   : nothing configured yet -- only reachable before the first configure of either kind.
 *   SINGLE : committed via inst_configure() -> the per-leg PRIMARY-only API
 *            (inst_configure / inst_start / inst_stop) is in force.
 *   SYSTEM : committed via configure_system() -> the whole-system domain API
 *            (configure_system / start_domain / start_all_domains / stop_domain /
 *            stop_all_domains) is in force.
 * configure_system() may full-recommit from ANY mode (NONE/SINGLE/SYSTEM) while CLOSED and
 * fully stopped -- it is the only escape from SINGLE ownership. Once SYSTEM, inst_configure()
 * is rejected (a system caller must stay transactional).
 */
typedef enum {
    TDM_CONFIG_MODE_NONE = 0,
    TDM_CONFIG_MODE_SINGLE,
    TDM_CONFIG_MODE_SYSTEM,
} tdm_config_mode_t;

// Private to this file BY DESIGN: the mode is not a public query. What callers see is the
// ERR_CONFIG_MODE verdict, so no consumer can branch on the mode and re-derive the policy.
static tdm_config_mode_t s_config_mode = TDM_CONFIG_MODE_NONE;

/*
 * Is this leg the stream's PRIMARY leg -- the only leg the per-leg SINGLE API may touch?
 *
 * On CK the primary leg is the block-timing reference (the row whose RX DMA block IRQ
 * drives the app callback), which is the same leg open() already derives the clock role
 * from. A non-primary (follower) leg is reachable only through the whole-system API, so
 * the per-leg calls reject it rather than half-configuring a co-clocked pair one leg at a
 * time -- the failure mode where SPI2 rides framing that SPI1's config never agreed to.
 */
static bool tdm_inst_is_primary( const tdm_spi_leg_t *leg )
{
    return ( leg != NULL ) && ( leg == tdm_stream_get_block_timing_master_leg( &s_stream ) );
}


// Is any leg currently streaming? Backs the close() and set_port() "not while streaming"
// guards. Asks every leg, not the block-timing reference: a follower left running is
// exactly the case those two guards exist to refuse, and is_running() would not see it.
static bool tdm_any_leg_running( void )
{
    const uint8_t n = s_stream.leg_count;
    for( uint8_t i = 0u; i < n; i++ )
    {
        if( s_stream.legs[i].running )
        {
            return true;
        }
    }
    return false;
}




//===========================================================
// Global Function
//===========================================================

// DMA global init/validate were removed in favor of the low-level DMA HAL.
// main() calls nora_dma_global_init() once at startup; the per-channel
// config below checks nora_dma_channel_config()/_enable() return values.


/*
 * Register the board/clock adapter used by this HAL.
 *
 * The table is stored by pointer, not copied, so the caller must provide a
 * static/long-lived object. Passing NULL returns the HAL to its board-free
 * default: no pin routing, no clock gate, and no clock-change events.
 *
 * Rejected (false, port UNCHANGED) while the port is already open()'d or any leg is
 * running: open() consumes the port hooks (clock/pins/CLC), so swapping the port after
 * open -- or under a live stream -- would leave the routed hardware disagreeing with the
 * registered hooks. Call it before open() (typically once at init).
 */
bool nora_spi_i2s_tdm_set_port( const nora_spi_i2s_tdm_port_t* port )
{
    if( s_stream.opened || tdm_any_leg_running() )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }
    s_stream.port = port;
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Report whether the external stream source is ready.
 *
 * This is a readiness gate, not a running-state check. With a registered port it
 * asks clock_source_ready(role); without a port it returns true so the HAL can
 * run as a self-clocked/no-gate transport.
 */
bool nora_spi_i2s_tdm_is_active( void )
{
    const tdm_stream_t *stream = &s_stream;

    // Stream-readiness gate routed through the clock port. No port (or no
    // clock_source_ready hook) => always ready (self-clocked, no external gate).
    // A board may wire this to an external audio-clock readiness signal.
    // Pass the configured role. Before the first configure(), treat the transport
    // explicitly as SLAVE instead of relying on enum zero-initialization.
    if( ( stream->port != NULL ) && ( stream->port->clock_source_ready != NULL ) )
    {
        const tdm_spi_leg_t *timing_leg = tdm_stream_get_block_timing_master_leg( stream );
        nora_spi_i2s_tdm_clock_role_t role =
            ( ( timing_leg != NULL ) && timing_leg->config_valid ) ? timing_leg->config.clock_role
                                                                    : NORA_SPI_I2S_TDM_CLOCK_SLAVE;
        return stream->port->clock_source_ready( role );
    }
    return true;
}


/*
 * Re-check the clock-readiness gate immediately before a start (arm).
 *
 * open() checks readiness ONCE; between open() and start() an external BCLK/FS could drop.
 * inst_start() calls this just before arming so a stream is not entered with the source
 * already gone. Same gate as is_active() / open(): keyed on the block-timing reference's
 * role and routed through the port. A self-clocked master (no port, or no
 * clock_source_ready hook) is always ready. Returns true = go.
 *
 * READINESS SCOPE (by design, same as canonical): ENGINE-WIDE and keyed on the block-timing
 * reference, NOT per leg -- a follower rides the reference's clock, so there is one source to
 * be ready. A per-leg readiness hook is not supported and should be revisited before this HAL
 * is used for genuinely independent async legs.
 *
 * A named one-line wrapper rather than calling is_active() at the start site, because the two
 * questions differ in intent and only this one is a gate: is_active() answers an application's
 * "may I try?", this answers the HAL's "am I still allowed to arm?". Canonical draws the same
 * line in the same place.
 */
static bool tdm_stream_ready_for_start( void )
{
    return nora_spi_i2s_tdm_is_active();
}


/*
 * Report the engine's running state = the block-timing reference instance (SPI1).
 *
 * Set by a successful inst_start() of the block-timing reference and cleared by its inst_stop(). It
 * is separate from is_active(), which only means the clock/source gate is ready. For
 * a specific instance use inst_get_status().running.
 */
bool nora_spi_i2s_tdm_is_running( void )
{
    return s_spi_legs[TDM_SPI_LEG_BLOCK_REF].running;
}


/*
 * Read and clear the next external-clock stop/resume event.
 *
 * Clock detection lives in the board adapter and is reached through the port
 * hook. The app consumes this edge to run a mute-bounded stop/reconfigure/restart
 * sequence; without a hook, the HAL reports NONE.
 */
nora_spi_i2s_tdm_clock_event_t nora_spi_i2s_tdm_consume_clock_event( void )
{
    const tdm_stream_t *stream = &s_stream;

    // Routed through the clock port. No port (or no hook) => NONE (no external
    // clock to detect). A board may wire this to an RB15/CN edge or equivalent event.
    if( ( stream->port != NULL ) && ( stream->port->consume_clock_event != NULL ) )
    {
        return stream->port->consume_clock_event();
    }
    return NORA_SPI_I2S_TDM_CLOCK_EVENT_NONE;
}


// The transport is RATE-AGNOSTIC: it moves 32-bit words at whatever BCLK/FS the
// configured BRG (master) or the external clock (slave) provides, and never derives
// anything from a sample-rate value. So the old rate API
// (is_supported_sample_rate / notify_sample_rate / get_sample_rate /
// set_rate_callback + the rate_state machine + rate-change callback) has been removed.
// Sample-rate POLICY is not a HAL property at all -- the supported-rate set is the
// product/board's (APP_SAMPLE_RATE_IS_SUPPORTED in the app layer, used by the CMSIS-SAI
// wrapper to validate ARM_SAI AUDIO_FREQ). Any runtime rate DETECTION (e.g. from a
// USB-audio clock notification or an FS measurement) and the stop->reconfigure->start
// it would drive belong in the application, not here.


/*
 * Number of SPI instances this build has (the size of the instance list in conf.h).
 * Pair with inst(i) to enumerate: for i in 0 .. instance_count()-1.
 */
uint8_t nora_spi_i2s_tdm_instance_count( void )
{
    return s_stream.leg_count;
}

/*
 * Return the handle for one SPI instance, or NULL if index is out of range.
 *
 * index is a leg index in list order (0 = the block-timing reference); the TDM_SPI_LEG_*
 * names are core-internal. The handle is the address of the static SPI leg descriptor.
 * spi1()..spi4() below are NOT wrappers over an index -- they search for a physical SPI.
 */
nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_inst( uint8_t index )
{
    if( index >= (uint8_t)TDM_SPI_LEG_COUNT )
    {
        return NULL;
    }
    return &s_spi_legs[index];
}

/*
 * Find the leg driving one LITERAL physical SPI, or NULL if no leg in this build does.
 *
 * This is the canonical spiN() semantic and it is deliberately a search, not an index:
 * spi2() must mean "the leg on SPI2" even if a future instance list puts SPI2 first, or
 * lists SPI3 without SPI2. The old form returned inst(TDM_SPI_LEG_SPI2), which was really
 * "the second row" and only agreed with the physical number because the default list
 * happens to be SPI1-then-SPI2 -- a coincidence, and one a reordered conf.h would break
 * silently. The loop is over at most TDM_SPI_INST_COUNT (3) rows and these accessors are
 * setup-path only, never in the block ISR.
 */
static nora_spi_i2s_tdm_inst_t* tdm_find_leg_by_phys_spi( tdm_spi_inst_t phys )
{
    for( uint8_t i = 0u; i < (uint8_t)TDM_SPI_LEG_COUNT; ++i )
    {
        if( s_spi_legs[i].spi_inst == phys )
        {
            return &s_spi_legs[i];
        }
    }
    return NULL;   // this build has no leg on that physical SPI
}

nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi1( void )
{
    return tdm_find_leg_by_phys_spi( TDM_SPI1 );
}

nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi2( void )
{
    return tdm_find_leg_by_phys_spi( TDM_SPI2 );
}

nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi3( void )
{
    // NULL unless a conf.h instance row actually targets TDM_SPI3, so on CK64MC105 -- which
    // has no SPI3 -- this is NULL simply because nothing targets it. Note that is a property
    // of the instance list, NOT an enforced one: hw.c keeps a ZEROED [TDM_SPI3] facts row on
    // parts without SPI3, so a conf.h that named TDM_SPI3 on MC105 would build and then fail
    // at runtime on NULL register pointers. Making that a hard #error belongs with the
    // conf.h-ownership work (phase 5), not here.
    return tdm_find_leg_by_phys_spi( TDM_SPI3 );
}

nora_spi_i2s_tdm_inst_t* nora_spi_i2s_tdm_spi4( void )
{
    // Declared for contract completeness only: no supported dsPIC33CK part has a 4th SPI
    // (there is no TDM_SPI4 in tdm_spi_inst_t), so there is nothing to search for. NULL for
    // an absent instance is exactly what the canonical contract specifies.
    return NULL;
}


//===========================================================
// Last-error diagnostic (debug aid; see the header contract). Plain last-writer-wins
// store -- NOT updated from the ISR hot path and NOT stream health.
//===========================================================
static volatile nora_spi_i2s_tdm_error_t s_last_error = NORA_SPI_I2S_TDM_ERR_NONE;

static inline void tdm_set_error( nora_spi_i2s_tdm_error_t err )
{
    s_last_error = err;
}

nora_spi_i2s_tdm_error_t nora_spi_i2s_tdm_get_last_error( void )
{
    return s_last_error;
}


/*
 * Return one instance's current writable TX ping-pong half, or NULL.
 *
 * For an app that produces one instance's output from ANOTHER instance's block callback
 * -- e.g. a co-clocked dual-codec demo where SPI1's callback fills BOTH its own and
 * SPI2's TX so the two codecs stay sample-aligned (same block, same frame), with no
 * cross-ISR handoff or ordering/race dependency. The returned half is the one NOT being
 * transmitted (same selection the instance's own block callback receives as dst). Only
 * valid while inst is running, and only meaningful when called at a block boundary --
 * co-clocked siblings share the ping-pong phase, so SPI2's writable half then matches
 * SPI1's. Returns NULL if inst is NULL/stopped or the half cannot be resolved; the
 * caller must NULL-check before writing.
 */
nora_tdm_slot_t* nora_spi_i2s_tdm_inst_tx_fill_ptr( nora_spi_i2s_tdm_inst_t* inst )
{
    nora_tdm_slot_t* dst = NULL;

    if( ( inst == NULL ) || !inst->running )
    {
        return NULL;
    }
    // Per-leg pong-half offset = slots * blk (runtime path; the hot ISR uses a literal).
    tdm_get_dest_ptr( nora_dma_read_src( inst->tx_dma_ch ), inst->tx_buffer,
                      (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames, &dst );
    return dst;
}


/*
 * Return one instance's TX ping-pong half by MIRRORING a reference instance's fill half,
 * instead of reading this instance's own live DMA position.
 *
 * For the co-clocked single-producer dual-codec demo: leg A's block callback fills BOTH
 * codecs from the SAME callback. Using inst_tx_fill_ptr(B) there reads B's live TX DMA and
 * returns "the half not being transmitted NOW" -- a snapshot sampled at callback start that
 * can go stale (B crosses its own block boundary while the long callback runs) -> the write
 * lands in the half B is transmitting -> tearing under high load. Mirroring removes the live
 * read: the returned half is whichever half of THIS instance corresponds to the reference's
 * just-handed fill half (`ref_fill_half`, i.e. A's `dst`). Because two co-clocked legs share
 * the ping-pong phase and equal block geometry, the mirrored half is the correct (safe,
 * not-transmitting) half for the WHOLE block -- deterministic, no live-DMA snapshot, A and B
 * sample-aligned (same generation, zero skew).
 *
 * Returns a typed result (see the header enum). On OK, *dst = the writable (not-transmitting)
 * target half. On BAD_ARGUMENT (NULL arg / stopped inst / ref_fill_half outside ref's buffer),
 * UNSAFE_ACTIVE_HALF (inst is transmitting the target half NOW) or UNRESOLVED_DMA_POSITION
 * (inst's live TX-DMA address is out of buffer range -- reload boundary / just-started /
 * fault), *dst=NULL and the caller must NOT write the mirrored leg this block. The target half
 * itself is deterministic (from ref_fill_half); the live-DMA read is only the secondary veto
 * that produces UNSAFE/UNRESOLVED.
 *
 * Slot units, not DMA elements. Offsets below are SLOT counts and the pointer arithmetic is on
 * nora_tdm_slot_t, so this backend's 16-bit TX DMA element never leaks out: the raw DMAxSRC
 * address advances twice per slot, and the only place that matters is the address RANGE tests,
 * which stay in the integer domain against &tx_buffer[n] and therefore convert for free.
 */
nora_spi_i2s_tdm_mirror_result_t nora_spi_i2s_tdm_inst_tx_fill_mirror(
        nora_spi_i2s_tdm_inst_t*       inst,
        const nora_spi_i2s_tdm_inst_t* ref,
        const nora_tdm_slot_t*         ref_fill_half,
        nora_tdm_slot_t**              dst )
{
    if( dst == NULL )
    {
        return NORA_TDM_MIRROR_BAD_ARGUMENT;
    }
    *dst = NULL;   // fail-closed default: only OK sets a non-NULL pointer
    if( ( inst == NULL ) || ( ref == NULL ) || ( ref_fill_half == NULL ) )
    {
        return NORA_TDM_MIRROR_BAD_ARGUMENT;
    }
    // inst and ref must be handles returned by this HAL's accessors (spiN()/inst(i)).
    // tdm_spi_leg_is_valid() checks each descriptor's local invariants (known SPI instance,
    // distinct RX/TX channels, non-NULL buffers) before use; it is not a defense against an
    // arbitrary bogus pointer. Reject on that check or a stopped inst as BAD_ARGUMENT.
    if( !tdm_spi_leg_is_valid( inst ) || !tdm_spi_leg_is_valid( ref ) || !inst->running )
    {
        return NORA_TDM_MIRROR_BAD_ARGUMENT;
    }
    const uint32_t ref_half  = (uint32_t)ref->geom_slots_per_fs  * ref->geom_block_frames;
    const uint32_t inst_half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;

    // Which half of ref is ref_fill_half? [base, base+ref_half) = ping (index 0); the pong half
    // starts at base+ref_half. Reject a pointer outside ref's [base, base+2*ref_half). Compare in
    // the integer domain (uintptr_t): comparing pointers into DIFFERENT array objects is undefined
    // in C, so a bogus ref_fill_half must be range-checked as an integer -- same idiom as
    // tdm_get_dest_ptr() and as the inst live-DMA address check below.
    const uintptr_t rbase = (uintptr_t)&ref->tx_buffer[ DMA_BUF_PING_POS ];
    const uintptr_t rmid  = (uintptr_t)&ref->tx_buffer[ ref_half ];
    const uintptr_t rend  = (uintptr_t)&ref->tx_buffer[ 2u * ref_half ];
    const uintptr_t raddr = (uintptr_t)ref_fill_half;
    if( ( raddr < rbase ) || ( raddr >= rend ) )
    {
        return NORA_TDM_MIRROR_BAD_ARGUMENT;
    }
    const bool     pong       = ( raddr >= rmid );
    const uint32_t target_off = pong ? inst_half : 0u;   // SLOTS: the half we would fill

    // Live safety veto. With a phase-locked start the mirrored (ref-fill) half is always inst's
    // NON-transmitting half. Two abnormal cases must NOT authorize a write:
    //   - live TX-DMA address OUT of inst's buffer (reload boundary / just-started / fault): the
    //     active half is UNRESOLVABLE -> fail-closed as UNRESOLVED (a caller tolerates a few
    //     consecutive as a transient and resyncs only if persistent).
    //   - live address IN range AND on the very half we'd fill: a real phase problem -> UNSAFE.
    const uintptr_t ibase = (uintptr_t)&inst->tx_buffer[ DMA_BUF_PING_POS ];
    const uintptr_t imid  = (uintptr_t)&inst->tx_buffer[ inst_half ];
    const uintptr_t iend  = (uintptr_t)&inst->tx_buffer[ 2u * inst_half ];
    // Out-of-line twin on purpose: this is a block-boundary API, not the RX-block ISR body, so
    // it follows inst_tx_fill_ptr() above rather than the `_hot` path (D6 -- by measured call
    // site, not by file).
    const uintptr_t iaddr = (uintptr_t)nora_dma_read_src( inst->tx_dma_ch );
    if( ( iaddr < ibase ) || ( iaddr >= iend ) )
    {
        return NORA_TDM_MIRROR_UNRESOLVED_DMA_POSITION;
    }
    const uint32_t active_off = ( iaddr >= imid ) ? inst_half : 0u;
    if( active_off == target_off )
    {
        return NORA_TDM_MIRROR_UNSAFE_ACTIVE_HALF;
    }
    *dst = inst->tx_buffer + target_off;
    return NORA_TDM_MIRROR_OK;
}


/*
 * Diagnostic: which TX ping-pong half is this instance's DMA CURRENTLY transmitting?
 *   0 = ping (first half), 1 = pong (second half), -1 = unresolved (stopped, or the live
 *   DMAxSRC snapshot is outside this buffer -- reload boundary / just-started / fault).
 *
 * Phase-probe use only (measure two co-clocked legs' ping-pong alignment for the
 * single-producer path). Reads the live DMA source address, so sample it at a deterministic
 * instant (e.g. a block-boundary ISR) and compare two co-clocked legs.
 */
int nora_spi_i2s_tdm_inst_tx_active_half( nora_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !inst->running )
    {
        return -1;
    }
    const uint32_t  half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;
    const uintptr_t base = (uintptr_t)&inst->tx_buffer[ DMA_BUF_PING_POS ];
    const uintptr_t mid  = (uintptr_t)&inst->tx_buffer[ half ];
    const uintptr_t end  = (uintptr_t)&inst->tx_buffer[ 2u * half ];
    const uintptr_t addr = (uintptr_t)nora_dma_read_src( inst->tx_dma_ch );

    if( ( addr < base ) || ( addr >= end ) )
    {
        return -1;
    }
    return ( addr >= mid ) ? 1 : 0;
}


/*
 * Diagnostic: the TX DMA's CURRENT read position as a SLOT offset into the full ping-pong
 * buffer [0, 2*half). Returns -1 if stopped or the live DMAxSRC snapshot is out of range.
 *
 * Finer than tx_active_half(): lets a phase probe measure the SUB-block sample offset between
 * two co-clocked legs (equal half but different position => a fixed offset that can still tear
 * a late cross-fill write). Sample at a deterministic instant and diff two legs.
 *
 * The unit is one SLOT, matching the contract, NOT this part's DMA element: the TX DMA moves
 * 16-bit wire words, so DMAxSRC advances TWICE per slot, and dividing by sizeof(nora_tdm_slot_t)
 * -- not by the element size -- is what makes a two-leg diff mean the same number of samples it
 * means on a family whose element is the whole slot. The truncating divide is deliberate: a
 * snapshot taken mid-slot (the low wire word already fetched) reports the slot it is inside.
 */
int32_t nora_spi_i2s_tdm_inst_tx_active_pos( nora_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !inst->running )
    {
        return -1;
    }
    const uint32_t  half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;
    const uintptr_t base = (uintptr_t)&inst->tx_buffer[ DMA_BUF_PING_POS ];
    const uintptr_t end  = (uintptr_t)&inst->tx_buffer[ 2u * half ];
    const uintptr_t addr = (uintptr_t)nora_dma_read_src( inst->tx_dma_ch );

    if( ( addr < base ) || ( addr >= end ) )
    {
        return -1;
    }
    return (int32_t)( ( addr - base ) / sizeof(nora_tdm_slot_t) );   // 0 .. 2*half-1
}


/*
 * Register or clear one SPI instance's audio-block callback.
 *
 * That instance's RX-block ISR calls this hook once per completed block. The update
 * is bracketed by briefly masking the instance's RX DMA IE so the ISR cannot observe
 * a torn cb/user pair.
 *
 * Returns false (and changes nothing) if the contract is violated: NULL inst, or the
 * instance is running and the call would CHANGE the (cb,user) pair -- the callback must
 * be registered before the leg is STARTED (inst_start() in SINGLE mode,
 * start_domain()/start_all_domains() in SYSTEM mode) and not swapped/cleared mid-stream.
 * This entry point itself has NO config-mode gate: a callback is per-instance state, not
 * part of the SINGLE/SYSTEM API split, so it is legal on any leg in either mode.
 * Re-registering
 * the identical (cb,user) while running is allowed (idempotent no-op -> true).
 */
bool nora_spi_i2s_tdm_set_block_callback( nora_spi_i2s_tdm_inst_t* inst,
                                               nora_spi_i2s_tdm_block_cb_t cb,
                                               void* user )
{
    bool     rxie_bak;

    if( inst == NULL )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }

    // While running, only a no-op re-register of the same pair is permitted.
    if( inst->running && ( cb != inst->block_cb || user != inst->block_user ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }

    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );

    inst->block_cb   = cb;
    inst->block_user = user;

    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Open the shared board/clock port for the engine, ONCE, before any instance is started.
 *
 * Takes NO role argument: the clock role handed to the port hooks is DERIVED from the
 * committed block-timing reference leg, so the pin/clock direction can never disagree with
 * the configured stream. Until phase 2 the app passed the role, and nothing tied the value
 * it passed to the role it had committed in inst_configure() -- removing the argument
 * removes that bug class rather than just a parameter, which is why the caller-visible
 * change is worth its two call sites.
 *
 * Brings up + checks the external clock and routes pins/CLC through the registered
 * port hooks (all optional). Returns false if the block-timing reference is not configured
 * (ERR_NOT_CONFIGURED -- configure before open), the external clock cannot be brought up /
 * is not ready, or a pin/CLC hook rejects the role -- the caller must then not start any
 * instance. With no port registered this is a no-op success (self-clocked). It does
 * NOT block waiting for a clock (single readiness check) and does NOT touch any
 * SPI/DMA -- per-instance start arms the hardware. Co-clocked followers ride the same
 * clock/pins, which is why one role is enough for the engine.
 *
 * Idempotent: a second open() while already open succeeds without re-running the port
 * hooks. That is not an optimisation -- the hooks have side effects (external-clock
 * bring-up, CLC engage), and re-running them under a stream armed against the first
 * routing is what idempotence is protecting against.
 */
bool nora_spi_i2s_tdm_open( void )
{
    const tdm_stream_t  *stream = &s_stream;
    const tdm_spi_leg_t *timing_leg;
    nora_spi_i2s_tdm_clock_role_t role;

    // Already open: do NOT re-run the port hooks (see above), just succeed.
    if( s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
        return true;
    }

    // Verify the shared-engine topology ONCE here, before any clock/pin bring-up:
    // exactly one block-timing reference (the first leg), distinct physical SPIs, and
    // distinct DMA channels across all legs. This catches a leg-table misconfig
    // (e.g. two legs on the same SPI, or a crossed DMA channel) that the per-leg
    // tdm_spi_leg_is_valid() check at configure/start cannot see.
    if( !tdm_stream_topology_is_valid( stream ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_TOPOLOGY );
        return false;
    }

    /*
     * Derive the clock role from the COMMITTED block-timing reference (never from a caller
     * argument). That leg MUST be configured: open() with an unconfigured reference is a
     * contract error, not a silent SLAVE default -- a silent default is precisely how a board
     * ends up pin-routed as a slave while its committed config says master.
     */
    timing_leg = tdm_stream_get_block_timing_master_leg( stream );
    if( ( timing_leg == NULL ) || !timing_leg->config_valid )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;
    }
    role = timing_leg->config.clock_role;

    if( stream->port != NULL )
    {
        if( ( stream->port->clock_source_init != NULL ) && !stream->port->clock_source_init( role ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_CLOCK_INIT );
            return false;   // external clock could not be brought up (e.g. unsupported role)
        }
        if( ( stream->port->clock_source_ready != NULL ) && !stream->port->clock_source_ready( role ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
            return false;   // clock not ready yet -- caller retries open() later
        }
        if( ( stream->port->configure_pins != NULL ) && !stream->port->configure_pins( role ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_PIN_CONFIG );
            return false;   // role this board cannot pin-route
        }
        if( ( stream->port->clc_passthrough != NULL ) && !stream->port->clc_passthrough( role ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_CLC );
            return false;
        }
    }
    s_stream.opened = true;   // port up: start/arm may now proceed
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Close the shared port after all instances are stopped.
 *
 * Deliberately a near-no-op: like stop(), the HAL does NOT tear down PPS/CLC routing
 * or the external clock -- other peripherals (or the next open()/start()) may depend
 * on them, and the port has no deinit hook. Provided for lifecycle symmetry and as
 * the place a future clock-deinit hook would run.
 *
 * Rejected (false, STAYS OPEN) while any leg is still running: closing under a live stream
 * would make the lifecycle state (closed) disagree with the hardware (SPI/DMA still on), and
 * the next inst_start() would then be refused as ERR_NOT_OPEN on a stream that never stopped.
 * Stop every leg first (inst_stop). On success the open flag is cleared, so the next start
 * needs a fresh open() -- that is what makes stop -> close -> open -> start a real restart
 * rather than a sequence of no-ops.
 */
bool nora_spi_i2s_tdm_close( void )
{
    if( tdm_any_leg_running() )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    s_stream.opened = false;   // a fresh open() is required before the next start/arm
    // No hardware teardown by design (see above).
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// The macro-derived default config lives in the platform layer -- see
// audio_app_board_get_default_config() in audio_app/. The core
// no longer fabricates a config from app macros; callers configure() explicitly.


/*
 * Store a validated configuration for ONE instance (declaration only; no HW write).
 *
 * This is the SINGLE-mode, PRIMARY-only per-leg entry. Rejected (false) if: the port is
 * already open()'d (ERR_ALREADY_OPEN -- configure BEFORE open, because open() now derives
 * the clock role from this very config and routing the pins for one role while the stored
 * config says the other is the disagreement phase 2 exists to make unreachable); the stream
 * was committed via configure_system() (mode == SYSTEM -> ERR_CONFIG_MODE; a system caller
 * stays transactional); inst is not the primary leg (ERR_CONFIG_MODE -- a non-primary leg is
 * reachable only through configure_system()); the instance is running or invalid; or cfg is
 * outside the supported wire-format envelope (NULL-safe). On success the config is stored and
 * the mode becomes SINGLE. Each leg carries its OWN clock role: this HAL does NOT force a
 * follower to SLAVE anywhere -- a follower is SLAVE because it was configured SLAVE, and the
 * stored role is what reaches the registers unchanged.
 *
 * The mode/primary gates sit AFTER the opened check, matching canonical: a call made while
 * RUNNING is answered ERR_ALREADY_OPEN, the same code it got before phase 3, so no existing
 * caller's diagnosis changes for a state that was already illegal.
 */
bool nora_spi_i2s_tdm_inst_configure( nora_spi_i2s_tdm_inst_t* inst,
                                           const nora_spi_i2s_tdm_config_t* cfg )
{
    // Reconfiguring a live instance would glitch (or tear) the framing mid-block; the
    // contract is stop -> configure -> start, so reject configure while running.
    if( inst == NULL )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    // Configure happens BEFORE open(): open() consumes this config to derive the clock role
    // and route pins/CLC, so configuring under an open port would desync HW from the config.
    if( s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }
    // Mode ownership: a SYSTEM-committed stream must be reconfigured only transactionally
    // (configure_system); and the per-leg API only ever addresses the PRIMARY leg. A
    // non-primary leg is configured exclusively through configure_system().
    if( s_config_mode == TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( inst->running )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    if( !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( cfg == NULL )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_ARGUMENT );
        return false;
    }
    if( !tdm_config_is_supported( inst, cfg ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
        return false;
    }
    // The leg's sync_domain (from the conf.h seed here; configure_system() will overwrite it)
    // must fit the 0..31 range the domain mask in phase 4's start_all_domains() can track --
    // guard the per-leg path too. See also the conf.h SYNC_DOMAIN #error and the leg-table
    // compile assert: this runtime check exists for a domain set by a future caller, not by
    // the macros, so it cannot be a compile assert alone.
    if( inst->sync_domain >= 32u )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_TOPOLOGY );
        return false;
    }

    inst->config       = *cfg;
    inst->config_valid = true;
    s_config_mode      = TDM_CONFIG_MODE_SINGLE;   // per-leg primary API now owns the config
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Transactional whole-system configure. setups[i] targets leg index i; setup_count must
 * equal the built leg count (TDM_SPI_LEG_COUNT). See the header for the contract.
 *
 * Two passes, all-or-nothing:
 *   1. PREFLIGHT (zero side effects) -- every leg must be a valid descriptor, STOPPED, and
 *      its stream must pass tdm_config_is_supported; each sync domain may hold at most one
 *      clock MASTER; and legs sharing a sync domain must agree on the frame interpretation
 *      (tdm_domain_framing_matches). Any failure rejects the whole set before a leg is touched.
 *   2. COMMIT -- only after a clean preflight, store each leg's config + sync_domain +
 *      config_valid together. Because preflight already validated everything, commit cannot
 *      fail, so there is never a partially-configured mix (SPI1-new + SPI2-old).
 *
 * NO MODE CHECK, deliberately, and this is the one place in the family that has none. A full
 * recommit is legal from NONE, SINGLE or SYSTEM: this call is the only way out of SINGLE
 * ownership, so gating it on the mode would strand a stream that had ever used inst_configure()
 * with no route to the domain API. The safety comes from the other two gates instead -- the port
 * must be CLOSED (ERR_ALREADY_OPEN) and every leg must be STOPPED -- so a live stream is
 * rejected, not silently reshaped. Do NOT "fix" the asymmetry: the *tm exerciser asserts that
 * a RUNNING stream answers ERR_ALREADY_OPEN here and not ERR_CONFIG_MODE, precisely so that
 * adding a mode gate breaks a test instead of quietly removing the escape hatch.
 *
 * The caller owns stop->configure->start: this does NOT stop a running transport (it
 * rejects one via the STOPPED preflight), keeping the call side-effect-free on rejection.
 */
bool nora_spi_i2s_tdm_configure_system( const nora_spi_i2s_tdm_leg_setup_t* setups,
                                             uint8_t setup_count )
{
    if( ( setups == NULL ) || ( setup_count != (uint8_t)TDM_SPI_LEG_COUNT ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_ARGUMENT );
        return false;
    }
    // Configure happens BEFORE open() (open() consumes the committed config). A full recommit
    // is allowed from ANY mode (NONE/SINGLE/SYSTEM) as long as the port is closed and every
    // leg is stopped (the STOPPED preflight below enforces the latter).
    if( s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }

    // 1a. PREFLIGHT: each leg valid, stopped, and its stream supported. Zero side effects.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || !tdm_spi_leg_is_valid( leg ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
            return false;
        }
        if( leg->running )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
            return false;
        }
        if( !tdm_config_is_supported( leg, &setups[i].stream ) )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
            return false;
        }
        // sync_domain must fit the 0..31 range that start_all_domains()'s 32-bit dedup/rollback
        // mask can track. A domain id >= 32 would be silently dropped from the started-mask, so
        // its legs could be started twice or skipped on rollback. Reject at configure (fail
        // closed) rather than misbehave at start. (This board uses domain 0 only; the guard is
        // for public reuse, and the per-leg path in inst_configure() checks it too.)
        if( setups[i].sync_domain >= 32u )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_TOPOLOGY );
            return false;
        }
    }

    // 1b. PREFLIGHT: at most one clock MASTER per sync domain (a domain has exactly one
    // clock source; two masters would fight for BCLK/FS -- and on CK they would also fight
    // for the single CLC1 FS generator).
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        if( setups[i].stream.clock_role != NORA_SPI_I2S_TDM_CLOCK_MASTER )
        {
            continue;
        }
        for( uint8_t j = (uint8_t)( i + 1u ); j < setup_count; j++ )
        {
            if( ( setups[j].sync_domain == setups[i].sync_domain ) &&
                ( setups[j].stream.clock_role == NORA_SPI_I2S_TDM_CLOCK_MASTER ) )
            {
                tdm_set_error( NORA_SPI_I2S_TDM_ERR_TOPOLOGY );
                return false;
            }
        }
    }

    // 1c. PREFLIGHT: legs sharing a sync domain are co-clocked on ONE BCLK/FS, so their frame
    // interpretation must be identical (a mismatch would read the shared clock differently on
    // each leg). See tdm_domain_framing_matches for the fields compared vs allowed to differ.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        for( uint8_t j = (uint8_t)( i + 1u ); j < setup_count; j++ )
        {
            if( ( setups[j].sync_domain == setups[i].sync_domain ) &&
                !tdm_domain_framing_matches( &setups[i].stream, &setups[j].stream ) )
            {
                tdm_set_error( NORA_SPI_I2S_TDM_ERR_TOPOLOGY );
                return false;
            }
        }
    }

    // 2. COMMIT: preflight guarantees success, so store all legs together (config,
    // sync_domain, config_valid) -- the topology table is now the single source of both
    // the stream and the sync domain.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        leg->config       = setups[i].stream;
        leg->sync_domain  = setups[i].sync_domain;
        leg->config_valid = true;
    }
    s_config_mode = TDM_CONFIG_MODE_SYSTEM;   // whole-system domain API now owns the config
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Read back ONE leg's committed setup: the config inst_configure() stored, plus the leg's
 * sync domain.
 *
 * PURE QUERY -- it deliberately does NOT call tdm_set_error(), not even on the failure paths.
 * A board port hook may call this from inside open() to route that leg's pins/CLC from the
 * committed clock role; if this query wrote the error channel it would erase the real failure
 * code the caller is about to read. "Returns false" is the whole report.
 *
 * The unconfigured leg returns false rather than a zeroed setup, because a zeroed clock_role
 * is a VALID role (SLAVE) -- a caller could not otherwise tell "not configured" from "slave".
 */
bool nora_spi_i2s_tdm_inst_get_setup( const nora_spi_i2s_tdm_inst_t* inst,
                                           nora_spi_i2s_tdm_leg_setup_t* setup )
{
    if( ( inst == NULL ) || ( setup == NULL ) || !inst->config_valid )
    {
        return false;
    }
    setup->stream      = inst->config;
    setup->sync_domain = inst->sync_domain;
    return true;
}




/*
 * Start ONE instance: arm its RX/TX DMA, then program + enable its SPI (triggers,
 * then module ON). The shared port must already be open()'d -- ENFORCED since phase 2
 * (ERR_NOT_OPEN), where before it was only documented: arming DMA/SPI with the port
 * unrouted enters SPIEN on a silently dead stream, so this fails closed.
 *
 * Returns false (instance left stopped, its HW rolled back) if the port is not open, the
 * clock source is no longer ready, it is not configured, already running, or DMA setup
 * fails. Does NOT touch the port or any other instance -- the caller orders multi-instance
 * starts (followers before the block-timing reference so all outputs are armed when the
 * block-timing reference's cadence begins).
 *
 * Validation order matches canonical: instance validity -> config mode -> primary -> opened ->
 * clock readiness -> configured -> already-running -> arm. opened is checked BEFORE readiness so
 * a start before open() reports ERR_NOT_OPEN rather than a readiness-hook verdict, and the
 * readiness RE-CHECK (the open->start drop window: open() checks once, and an external BCLK/FS
 * can vanish in between) only runs once open() has already brought the source up.
 *
 * The mode gate sits BEFORE opened -- the opposite order from inst_configure(), deliberately.
 * Under a SYSTEM-committed stream (or for a follower leg) this call is one the caller may not
 * make AT ALL, so answering ERR_NOT_OPEN would send them to open() to fix a call that open()
 * cannot make legal. Under a NEVER-configured stream (mode NONE) the same asymmetry shows: a
 * start before the first configure answers ERR_CONFIG_MODE, where after a configure+stop it
 * answers ERR_NOT_OPEN -- the same call, two verdicts, which is what makes the gate observable.
 */
bool nora_spi_i2s_tdm_inst_start( nora_spi_i2s_tdm_inst_t* inst )
{
    // Gate FIRST -- do not arm DMA/SPI unless this instance is actually going to run.
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( ( s_config_mode != TDM_CONFIG_MODE_SINGLE ) || !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }
    if( !tdm_stream_ready_for_start() )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
        return false;
    }

    // arm (everything up to but excluding SPIEN) then go (SPIEN). For a single leg the two
    // are adjacent, so this is exactly the old sequence; the split exists so start_domain()
    // can arm every leg first and release SPIEN back-to-back at the end.
    if( !tdm_inst_arm( inst ) )
    {
        return false;   // arm() already wrote the error and rolled its own HW back
    }
    tdm_inst_go( inst );

    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * ARM one leg: everything the start needs EXCEPT turning the SPI module on.
 *
 * On success the leg's DMA channels are enabled and armed, its SPI registers are programmed,
 * its DMA-trigger events are on, the CLC 50%-FS generator is engaged (or released), and
 * SPIEN is still OFF -- so no clock/FS edge has been emitted and running is still false.
 * tdm_inst_go() is the only thing that finishes it.
 *
 * Validation here is the part that is per-LEG: BAD_INSTANCE -> NOT_CONFIGURED ->
 * ALREADY_RUNNING -> NOT_OPEN -> UNSUPPORTED_CONFIG -> the statl resolve. The stream-wide
 * gates (config mode, clock readiness) belong to the caller, because a domain start checks
 * them ONCE for the group rather than once per leg. NOT_OPEN is re-checked anyway: it is the
 * one stream-wide condition whose violation would arm a silently dead stream, and both
 * callers are cheap to double-check.
 *
 * Only inst_start()'s single-leg path can reach the ordering visible from one leg, so the
 * error-priority seen by a one-leg caller is unchanged from phase 3 by construction.
 *
 * The statl NULL check is a CK-specific seam and lives HERE, not in the caller: the pointer
 * must be published before either DMA channel or SPIEN is enabled, because the generated
 * vector reads it non-volatile. Arming is the last moment that is still race-free.
 */
static bool tdm_inst_arm( nora_spi_i2s_tdm_inst_t* inst )
{
    nora_spi_i2s_tdm_config_t eff_cfg;

    // config_valid implies the config already passed configure()'s envelope check.
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( !inst->config_valid )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;
    }
    if( inst->running )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    if( !s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }
    // Resolve the register-level config (a validated copy -- no role override; see the
    // helper's own comment).
    if( !tdm_spi_leg_get_effective_config( inst, &eff_cfg ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
        return false;
    }

    /*
     * Resolve the physical SPI's status register before enabling either DMA
     * channel or SPIEN.  The generated vector cannot run before that point,
     * so the non-volatile pointer publication is race-free.  A valid static
     * leg must always resolve; retain this check so a future bad leg-table row
     * fails safely at start rather than dereferencing NULL in an ISR.
     */
    inst->spi_statl = nora_spi_i2s_tdm_hw_get_statl( inst->spi_inst );
    if( inst->spi_statl == NULL )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }

    // Fresh diagnostics + a deterministic first block for this instance.
    nora_spi_i2s_tdm_diag_reset( &inst->diag );
    tdm_inst_clear_buffers( inst );

    // Arm this instance's RX/TX DMA; on failure roll back ITS DMA/SPI so a partial
    // start leaves no channel with CHEN/IE set and the SPI off.
    if( !nora_spi_i2s_tdm_hw_dma_config( inst->spi_inst,
                                              inst->rx_dma_ch, inst->tx_dma_ch,
                                              inst->rx_buffer, inst->tx_buffer,
                                              inst->buffer_slot_count ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_DMA_CONFIG );
        goto fail;
    }

    // Program + enable this instance's SPI: registers, then DMA-trigger events, then
    // the module ON (ON is the last step, after the port pins/CLC are routed).
    nora_spi_i2s_tdm_hw_apply_config( inst->spi_inst, &eff_cfg );
    nora_spi_i2s_tdm_hw_dma_trigger_enable( inst->spi_inst, true );

    // FS_50PCT on a TDM master: the SPI emits a half-frame marker (set by apply_config);
    // engage CLC1 to toggle it into a 50%-duty FS on the same FS pin BEFORE the module
    // turns on, so the very first marker is captured. FS_PULSE and any slave (FS is an input)
    // need no CLC -- release it in case this instance held it before. I2S is absent from this
    // condition on purpose: I2S + MASTER + FS_50PCT never reaches start(), configure() having
    // already refused it (its FRMSYPW=1 pulse is 25%, not 50%, on a 16-bit wire).
    if( ( eff_cfg.clock_role   == NORA_SPI_I2S_TDM_CLOCK_MASTER ) &&
        ( eff_cfg.format == NORA_SPI_I2S_TDM_FORMAT_TDM )  &&
        ( eff_cfg.fs_shape == NORA_SPI_I2S_TDM_FS_50PCT ) )
    {
        const nora_spi_i2s_tdm_fs_clc_result_t clc =
            nora_spi_i2s_tdm_fs_clc_engage( inst->spi_inst );
        if( clc != NORA_SPI_I2S_TDM_FS_CLC_OK )
        {
            // BUSY = CLC1 already owned by another instance/domain; NO_FS_PIN = FS not on a
            // physical pin (or no CLC1 on this part).
            tdm_set_error( ( clc == NORA_SPI_I2S_TDM_FS_CLC_BUSY )
                               ? NORA_SPI_I2S_TDM_ERR_CLC
                               : NORA_SPI_I2S_TDM_ERR_PIN_CONFIG );
            goto fail;
        }
    }
    else
    {
        nora_spi_i2s_tdm_fs_clc_release( inst->spi_inst );
    }

    // Armed. SPIEN deliberately still OFF -- tdm_inst_go() releases it.
    return true;

fail:
    tdm_inst_soft_stop_dma( inst );
    tdm_inst_clear_dma_flags( inst );
    nora_spi_i2s_tdm_hw_soft_stop( inst->spi_inst );
    nora_spi_i2s_tdm_hw_irq_clear_flags( inst->spi_inst );
    inst->running = false;
    return false;
}


/*
 * GO: release SPIEN on an already-armed leg. Nothing else, and no validation -- the caller
 * armed this leg and therefore already knows it may run.
 *
 * Deliberately not public. The whole value of the split is that arm() can be called for
 * several legs and then go() for each in a chosen order (followers first, block-timing
 * master last), and that ordering guarantee only holds if no external caller can interleave.
 * hw_module_enable() is already the SPIEN-only writer, so no new hw primitive is needed.
 *
 * Cannot fail: SPIEN is a single bit write and the leg's error state was set by the caller.
 */
static void tdm_inst_go( nora_spi_i2s_tdm_inst_t* inst )
{
    nora_spi_i2s_tdm_hw_module_enable( inst->spi_inst, true );
    inst->running = true;
}




/*
 * Stop ONE instance and make its next start deterministic.
 *
 * SoftStop policy (per instance): does NOT stop DMACONbits.ON (shared controller) or
 * change PPS/CLC routing; stops only this instance's SPI module + DMA channels, masks
 * its DMA IRQs first so its ISR cannot refill mid-teardown, clears its pending
 * status, and clears its buffers so a restart is silent. Safe to call when stopped.
 *
 * Returns false for a bad handle (NULL / invalid leg) -- the case the void version already
 * refused to act on but could not report -- and with ERR_CONFIG_MODE when the stream was not
 * committed through inst_configure() or inst is not the primary leg (a SYSTEM-committed stream
 * is torn down through stop_domain/stop_all_domains, not one leg at a time). Otherwise the
 * teardown is idempotent, so a valid primary handle always returns true.
 *
 * Consequence, recorded rather than fixed: a stop before the FIRST configure (mode NONE) now
 * returns false/ERR_CONFIG_MODE where it used to return true. The only such caller is
 * wm8904_audio_stop(), which already discards the result with (void) -- and "you never
 * configured anything" is a more honest answer than a silent success.
 */
bool nora_spi_i2s_tdm_inst_stop( nora_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( ( s_config_mode != TDM_CONFIG_MODE_SINGLE ) || !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }

    tdm_inst_stop_impl( inst );
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * The teardown itself, with NO validation and NO error write -- the caller has already
 * decided this leg may be stopped.
 *
 * Extracted so the group teardowns coming in phase 4 (stop_domain / stop_all_domains, and
 * start_domain's rollback path) tear a leg down through the SAME register order as
 * inst_stop() instead of a second copy of it. While inst_stop() is the only caller the
 * compiler inlines it back, so this costs nothing until phase 4 gives it a second caller.
 *
 * Idempotent by construction: every step is an unconditional "off"/clear, so stopping an
 * already-stopped leg is harmless -- which is what lets inst_stop() always report true for
 * a valid handle.
 */
static void tdm_inst_stop_impl( nora_spi_i2s_tdm_inst_t* inst )
{
    // Mark stopped up front so a per-instance running query sees the transition.
    inst->running = false;

    // DMA IRQs off + channels off first (no refill), then SPI module + triggers off.
    tdm_inst_soft_stop_dma( inst );
    nora_spi_i2s_tdm_hw_soft_stop( inst->spi_inst );

    // Release the CLC1 50%-FS generator if this instance owned it: the flip-flop is disabled
    // AND the external FS pad is routed back from CLC1OUT to its original FRMSYNC (SSx), so a
    // subsequent FS_PULSE start finds the SPI driving that pad directly again. The two
    // internal routes (FRMSYNC -> RPV0, CLCINA <- RPV0) are left in place -- they reach no pad
    // and re-engage re-asserts them. No-op if this instance did not own the CLC.
    nora_spi_i2s_tdm_fs_clc_release( inst->spi_inst );

    // Clear pending status/flags, then buffers, before the next start.
    tdm_inst_clear_dma_flags( inst );
    nora_spi_i2s_tdm_hw_irq_clear_flags( inst->spi_inst );
    tdm_inst_clear_buffers( inst );
}




/*
 * Do two legs sharing a sync domain agree on the BCLK/FS frame interpretation?
 *
 * Co-clocked legs (same sync_domain) ride ONE shared bit/frame clock, so the fields that
 * define HOW that shared clock is read must be identical on every member -- otherwise the
 * legs would sample the same wires with different framing. Compared: format, word_bits,
 * slots_per_fs, block_frames, fs_coincides_first_bclk (SPIFE), bclk_idle_high (CKP),
 * bclk_change_on_active_to_idle (CKE), AND fs_shape. fs_shape is compared because for I2S it
 * maps to FRMSYPW regardless of clock role (hw: FS_50PCT+I2S -> FRMSYPW=1, FS_PULSE -> 0), so
 * two co-clocked I2S legs with different fs_shape would read the SAME FS with different pulse
 * widths. Deliberately NOT compared (may legitimately differ per leg): clock_role (exactly one
 * master drives the shared clock, the rest are slaves), brg (a slave ignores it), and
 * mclk_enable. IGNROV/IGNTUR are HAL-fixed policies, not per-leg config fields.
 *
 * On CK the master-side FS_50PCT generator is CLC1 + virtual pin RPV0 (AK's sibling uses
 * CLC10 + RPV8), and it is a single shared resource -- which is why fs_clc_engage() can answer
 * BUSY, and why two masters in one domain are rejected before it is ever asked.
 */
static bool tdm_domain_framing_matches( const nora_spi_i2s_tdm_config_t* a,
                                        const nora_spi_i2s_tdm_config_t* b )
{
    return ( a->format                        == b->format ) &&
           ( a->word_bits                     == b->word_bits ) &&
           ( a->slots_per_fs                  == b->slots_per_fs ) &&
           ( a->block_frames                  == b->block_frames ) &&
           ( a->fs_coincides_first_bclk       == b->fs_coincides_first_bclk ) &&
           ( a->bclk_idle_high                == b->bclk_idle_high ) &&
           ( a->bclk_change_on_active_to_idle == b->bclk_change_on_active_to_idle ) &&
           ( a->fs_shape                      == b->fs_shape );
}


/*
 * Side-effect-free classification of one sync domain, shared by start_domain() (its preflight)
 * and start_all_domains() (its whole-set preflight). A sync domain is a phase-locked UNIT whose
 * INVARIANTS are re-checked at START -- not only in configure_system() -- because a member may
 * have been (re)configured via the per-leg path afterwards. Sets *err ONLY for INVALID.
 *   INVALID     : an unconfigured member (NOT_CONFIGURED), >1 clock MASTER (TOPOLOGY), a
 *                 same-domain framing disagreement (TOPOLOGY), or no member at all (BAD_INSTANCE).
 *   STOPPED     : every member configured + stopped -> startable.
 *   ALL_RUNNING : every member already running -> a start is an idempotent no-op.
 *   PARTIAL     : some-but-not-all members running -> reject WITHOUT teardown (a re-assert must
 *                 not kill live audio; a half-running domain is a foreign/inconsistent state).
 */
typedef enum {
    TDM_DOMAIN_INVALID = 0,
    TDM_DOMAIN_STOPPED,
    TDM_DOMAIN_ALL_RUNNING,
    TDM_DOMAIN_PARTIAL,
} tdm_domain_state_t;

static tdm_domain_state_t tdm_domain_classify( uint8_t domain,
                                               nora_spi_i2s_tdm_error_t *err )
{
    const uint8_t        n       = nora_spi_i2s_tdm_instance_count();
    const tdm_spi_leg_t *ref     = NULL;
    uint8_t              members = 0u;
    uint8_t              masters = 0u;
    uint8_t              running = 0u;

    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t *leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) )
        {
            continue;
        }
        members++;
        if( !leg->config_valid )
        {
            *err = NORA_SPI_I2S_TDM_ERR_NOT_CONFIGURED;
            return TDM_DOMAIN_INVALID;
        }
        if( leg->config.clock_role == NORA_SPI_I2S_TDM_CLOCK_MASTER )
        {
            masters++;
        }
        if( leg->running )
        {
            running++;
        }
        if( ref == NULL )
        {
            ref = leg;
        }
        else if( !tdm_domain_framing_matches( &ref->config, &leg->config ) )
        {
            *err = NORA_SPI_I2S_TDM_ERR_TOPOLOGY;
            return TDM_DOMAIN_INVALID;
        }
    }
    if( members == 0u )
    {
        *err = NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE;
        return TDM_DOMAIN_INVALID;
    }
    if( masters > 1u )
    {
        *err = NORA_SPI_I2S_TDM_ERR_TOPOLOGY;
        return TDM_DOMAIN_INVALID;
    }
    if( running == 0u )
    {
        return TDM_DOMAIN_STOPPED;
    }
    return ( running == members ) ? TDM_DOMAIN_ALL_RUNNING : TDM_DOMAIN_PARTIAL;
}


/*
 * Mode-agnostic teardown of every leg in one sync domain (idempotent; safe on stopped legs).
 *
 * PRIVATE, and deliberately WITHOUT a config-mode gate, so start_domain()/start_all_domains()
 * can use it as their internal rollback and the public SYSTEM wrappers can use it as their
 * teardown. Every member goes down through the SAME per-leg register order as inst_stop().
 */
static void tdm_stop_domain_impl( uint8_t domain )
{
    const uint8_t n = nora_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg != NULL ) && ( leg->sync_domain == domain ) )
        {
            tdm_inst_stop_impl( leg );
        }
    }
}


/*
 * Mode-agnostic teardown of every instance, all sync domains. PRIVATE counterpart of the public
 * SYSTEM wrapper. Covers a future SPI3/SPI4 leg in any domain automatically because it walks the
 * leg table rather than a domain list. Idempotent; safe on already-stopped legs.
 */
static void tdm_stop_all_domains_impl( void )
{
    const uint8_t n = nora_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( leg != NULL )
        {
            tdm_inst_stop_impl( leg );
        }
    }
}


/*
 * Start every config_valid leg of one sync domain PHASE-LOCKED: (1) ARM all members, then
 * (2) release SPIEN back-to-back -- non-MASTER (slave/follower) legs first, the clock-MASTER
 * leg LAST. The adjacent go() calls make the members' ping-pong DMAs latch the same FS edge
 * (an external-FS domain has 0 masters -> all slaves go back-to-back; an internal-FS domain's
 * master starts its BCLK/FS only after the slaves are armed and listening).
 *
 * SYSTEM-mode API. Non-destructive: a fully-running domain is idempotent success; a partial or
 * invalid domain is rejected WITHOUT teardown -- a re-assert must never cut live audio, and a
 * half-running domain is a foreign state this call has no mandate to "repair". Returns false and
 * rolls back only THIS call's arms if a leg fails to arm. open() must have run first; this call
 * does not open anything.
 *
 * Gate order: CONFIG_MODE -> NOT_OPEN -> classify -> CLOCK_NOT_READY. Readiness is re-checked
 * ONCE for the group, after the classifier says STOPPED and immediately before arming, which is
 * the open->start drop window inst_start() re-checks for a single leg.
 */
bool nora_spi_i2s_tdm_start_domain( uint8_t domain )
{
    const uint8_t                 n = nora_spi_i2s_tdm_instance_count();
    nora_spi_i2s_tdm_error_t err;

    // SYSTEM-mode ownership: domain start is only for a configure_system()-committed stream.
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    // open() (shared clock/pins/CLC + readiness) MUST have run first.
    if( !s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }

    // PREFLIGHT (zero side effects) via the shared classifier.
    const tdm_domain_state_t state = tdm_domain_classify( domain, &err );
    if( state == TDM_DOMAIN_INVALID )
    {
        tdm_set_error( err );
        return false;   // no side effects (do NOT stop_domain on a preflight reject)
    }
    if( state == TDM_DOMAIN_ALL_RUNNING )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
        return true;    // already fully up -> idempotent no-op success
    }
    if( state == TDM_DOMAIN_PARTIAL )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;   // partial running -> reject, leave the domain as-is
    }

    // STOPPED and about to arm: re-check the clock-readiness gate (the open->start drop window).
    if( !tdm_stream_ready_for_start() )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
        return false;
    }

    // (1) ARM every member. Preflight guaranteed all are stopped + configured, so a failure
    //     here only rolls back what THIS call armed (the teardown is idempotent on stopped legs).
    for( uint8_t i = 0u; i < n; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) )
        {
            continue;
        }
        if( !tdm_inst_arm( leg ) )
        {
            tdm_stop_domain_impl( domain );   // roll back only this call's arms (mode-agnostic)
            return false;                     // arm() already wrote the error
        }
    }

    // (2a) GO the non-master legs first (adjacent SPIEN releases).
    for( uint8_t i = 0u; i < n; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) || !leg->config_valid )
        {
            continue;
        }
        if( leg->config.clock_role != NORA_SPI_I2S_TDM_CLOCK_MASTER )
        {
            tdm_inst_go( leg );
        }
    }
    // (2b) then the clock-MASTER leg LAST -- its BCLK/FS starts after the slaves listen.
    for( uint8_t i = 0u; i < n; i++ )
    {
        tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) || !leg->config_valid )
        {
            continue;
        }
        if( leg->config.clock_role == NORA_SPI_I2S_TDM_CLOCK_MASTER )
        {
            tdm_inst_go( leg );
        }
    }
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Public SYSTEM-mode stop of one sync domain. Rejects (false, HW unchanged) unless the stream
 * was committed via configure_system() -- a SINGLE-mode stream tears down through inst_stop().
 *
 * Symmetric with start_domain() on the "unknown domain" question: an out-of-range (>= 32) or
 * MEMBER-LESS domain is ERR_BAD_INSTANCE, not a silent success. Stopping something that does not
 * exist is a caller bug worth reporting, and reporting it in both directions means a typo'd
 * domain id cannot read as "stopped fine". An existing but already-stopped domain is idempotent
 * true.
 */
bool nora_spi_i2s_tdm_stop_domain( uint8_t domain )
{
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( domain >= 32u )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    // Reject a domain with no member leg (an unknown id), mirroring start_domain()'s
    // members == 0 check inside the classifier.
    uint8_t       members = 0u;
    const uint8_t n       = nora_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t *leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg != NULL ) && ( leg->sync_domain == domain ) )
        {
            members++;
        }
    }
    if( members == 0u )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    tdm_stop_domain_impl( domain );
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Start every sync domain present in the leg table (each once). Start/rollback bookkeeping is
 * per-domain and there is no cross-domain start-ordering constraint here (NOTE: this is NOT full
 * independence -- source readiness is engine-wide / primary-leg-gated and shared resources such as
 * the FS_50PCT CLC and the board clock port are not per-domain; see tdm_stream_ready_for_start()).
 * SYSTEM-mode API. TWO PASSES so a later domain's failure can never tear down a domain that was
 * ALREADY running before this call (nor one this call did not touch):
 *   Pass 1 (side-effect-free): classify every DISTINCT domain. If ANY is PARTIAL or INVALID,
 *           reject the whole call touching NOTHING. Record which domains are STOPPED (startable).
 *           ALL_RUNNING domains are left running and are NOT recorded (never rolled back).
 *   Pass 2: start only the STOPPED domains, tracking newly_started_mask = domains THIS call
 *           actually started. On any failure, roll back ONLY newly_started_mask -- pre-existing
 *           running domains and untouched domains are preserved.
 * Returns false + ERR_NOT_CONFIGURED if no domain is configured. open() must run first.
 */
bool nora_spi_i2s_tdm_start_all_domains( void )
{
    const uint8_t                 n            = nora_spi_i2s_tdm_instance_count();
    uint32_t                      seen_mask    = 0u;   // distinct domains examined
    uint32_t                      stopped_mask = 0u;   // startable (all-stopped) domains
    nora_spi_i2s_tdm_error_t err;

    // SYSTEM-mode ownership + open() precondition.
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !s_stream.opened )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }

    // PASS 1: classify every distinct configured domain, side-effect-free. Any PARTIAL/INVALID
    // domain rejects the whole call before a single leg is touched.
    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t* leg = nora_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || !leg->config_valid )
        {
            continue;
        }
        const uint8_t dom = leg->sync_domain;
        if( ( dom >= 32u ) || ( ( seen_mask & ( 1uL << dom ) ) != 0u ) )
        {
            continue;   // >=32 is rejected at configure; skip duplicates
        }
        seen_mask |= ( 1uL << dom );

        const tdm_domain_state_t state = tdm_domain_classify( dom, &err );
        if( state == TDM_DOMAIN_INVALID )
        {
            tdm_set_error( err );
            return false;   // touch nothing
        }
        if( state == TDM_DOMAIN_PARTIAL )
        {
            tdm_set_error( NORA_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
            return false;   // touch nothing (do NOT tear a half-running domain down)
        }
        if( state == TDM_DOMAIN_STOPPED )
        {
            stopped_mask |= ( 1uL << dom );
        }
        // ALL_RUNNING: leave it running, not recorded -> never rolled back.
    }
    if( seen_mask == 0u )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;   // no configured domain to start
    }

    // PASS 2: start only the STOPPED domains; roll back ONLY what this call started.
    uint32_t newly_started_mask = 0u;
    for( uint8_t dom = 0u; dom < 32u; dom++ )
    {
        if( ( stopped_mask & ( 1uL << dom ) ) == 0u )
        {
            continue;
        }
        if( !nora_spi_i2s_tdm_start_domain( dom ) )
        {
            for( uint8_t d = 0u; d < 32u; d++ )
            {
                if( ( newly_started_mask & ( 1uL << d ) ) != 0u )
                {
                    tdm_stop_domain_impl( d );   // mode-agnostic; roll back only newly-started
                }
            }
            return false;   // pre-existing running domains untouched
        }
        newly_started_mask |= ( 1uL << dom );
    }
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Public SYSTEM-mode stop of every sync domain. Domain-level teardown counterpart to
 * start_all_domains() so callers never enumerate individual logical legs. Rejects
 * (false, HW unchanged) unless the stream was committed via configure_system() (mode==SYSTEM);
 * a SINGLE-mode stream tears down through inst_stop(). Idempotent success otherwise.
 */
bool nora_spi_i2s_tdm_stop_all_domains( void )
{
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( NORA_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    tdm_stop_all_domains_impl();
    tdm_set_error( NORA_SPI_I2S_TDM_ERR_NONE );
    return true;
}




/*
 * Snapshot ONE instance's ISR load monitor.
 *
 * Values are updated from that instance's RX-block ISR, so its RX DMA IE is briefly
 * masked while reading the 32-bit counters on the 16-bit core. Returns false (NULL
 * args), or until at least one timed event exists and the high-resolution timer was
 * initialized. clear_peak resets min/max/event afterward.
 */
bool nora_spi_i2s_tdm_inst_get_load( nora_spi_i2s_tdm_inst_t* inst,
                                          nora_spi_i2s_tdm_load_t* monitor,
                                          bool clear_peak )
{
    bool     rxie_bak;
    bool     valid;

    if( ( inst == NULL ) || ( monitor == NULL ) )
    {
        return false;
    }

    // The diag counters are updated in this instance's RX-block ISR; mask its IE so
    // the snapshot + clear are consistent against the 16-bit core's non-atomic 32-bit
    // reads. The diag module itself does no masking.
    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );
    valid    = nora_spi_i2s_tdm_diag_get_load( &inst->diag, monitor, clear_peak );
    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );

    // Counts-to-microseconds AFTER the IE is restored. Three 64-bit divides inside
    // the mask made the window long enough to visibly delay the block ISR (paired
    // long/short block intervals summing to exactly two nominal periods, every time
    // the foreground sampled the load). The mask now covers only the raw snapshot
    // and clear, which is all that needs to be atomic against the ISR.
    nora_spi_i2s_tdm_diag_load_convert( monitor );

    return valid;
}


/*
 * Snapshot ONE instance's health/status.
 *
 * block_count, block_deadline_miss_count, load, and running are THIS instance's.
 * active is engine-wide (the shared clock/source readiness gate).
 */
bool nora_spi_i2s_tdm_inst_get_status( nora_spi_i2s_tdm_inst_t* inst,
                                            nora_spi_i2s_tdm_status_t* status,
                                            bool clear_peak )
{
    bool     rxie_bak;

    if( ( inst == NULL ) || ( status == NULL ) )
    {
        return false;
    }

    status->active  = nora_spi_i2s_tdm_is_active();   // shared clock/source readiness gate
    status->running = inst->running;                       // this instance's running state

    // 32-bit reads on a 16-bit core are not atomic vs this instance's RX-block ISR; mask briefly.
    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );
    nora_spi_i2s_tdm_diag_read_counts( &inst->diag,
                                            &status->block_count,
                                            &status->block_deadline_miss_count );
    // HW health counters read under the same mask (plain field copies, mirroring AK).
    status->rx_dma_overrun_count      = inst->diag.rx_dma_overrun_count;
    status->rx_dma_other_irq_count    = inst->diag.rx_dma_other_irq_count;
    status->rx_dma_last_status        = inst->diag.rx_dma_last_status;
    status->err_rov_block_count       = inst->diag.err_rov_block_count;
    status->err_tur_block_count       = inst->diag.err_tur_block_count;
    status->err_frm_block_count       = inst->diag.err_frm_block_count;
    status->frmerr_consecutive_blocks = inst->diag.frmerr_consecutive_blocks;
    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );

    // load monitor (does its own RX-IE guard; honours clear_peak)
    (void)nora_spi_i2s_tdm_inst_get_load( inst, &status->load, clear_peak );

    return true;
}


//===========================================================
// Engine-wide TDMsum profiler -- public wrappers.
//
// The profiler state is shared by EVERY leg's RX-block ISR, so a per-leg mask is not enough:
// each wrapper masks EVERY configured leg's RX DMA IE around the profiler call (the same
// bracket the per-leg readers use, generalised to all legs). The raw profiler ops in
// nora_spi_i2s_tdm_dspic33ck_diag.c do no masking themselves.
//
// The whole group -- wrappers, all-leg masking helpers, and the ISR enter/exit hooks in
// tdm_rx_block() -- is compiled only when NORA_TDM_SUMPROF is 1 (see
// nora_spi_i2s_tdm_conf.h).
//===========================================================
#if NORA_TDM_SUMPROF

// Disable every configured leg's RX DMA IE, saving prior enables into `bak[]` (indexed by leg).
static inline void tdm_all_rx_ie_disable( bool bak[TDM_SPI_LEG_COUNT] )
{
    uint8_t i;
    for( i = 0u; i < s_stream.leg_count; i++ )
    {
        bak[i] = tdm_rx_ie_disable( s_stream.legs[i].rx_dma_ch );
    }
}

// Restore every configured leg's RX DMA IE from `bak[]` (re-arms only those previously enabled).
static inline void tdm_all_rx_ie_restore( const bool bak[TDM_SPI_LEG_COUNT] )
{
    uint8_t i;
    for( i = 0u; i < s_stream.leg_count; i++ )
    {
        tdm_rx_ie_restore( s_stream.legs[i].rx_dma_ch, bak[i] );
    }
}

void nora_spi_i2s_tdm_tdmsum_configure( uint32_t window_period_ticks )
{
    bool bak[TDM_SPI_LEG_COUNT];
    const uint32_t now = nora_high_res_timer_get_count();

    tdm_all_rx_ie_disable( bak );
    nora_spi_i2s_tdm_dspic33ck_sumprof_configure( now, window_period_ticks );
    tdm_all_rx_ie_restore( bak );
}

void nora_spi_i2s_tdm_tdmsum_reset( void )
{
    bool bak[TDM_SPI_LEG_COUNT];
    const uint32_t now = nora_high_res_timer_get_count();

    tdm_all_rx_ie_disable( bak );
    nora_spi_i2s_tdm_dspic33ck_sumprof_reset( now );
    tdm_all_rx_ie_restore( bak );
}

bool nora_spi_i2s_tdm_tdmsum_get( nora_spi_i2s_tdm_tdmsum_t* out, bool clear_peak )
{
    bool bak[TDM_SPI_LEG_COUNT];

    if( out == NULL )
    {
        return false;
    }

    tdm_all_rx_ie_disable( bak );
    nora_spi_i2s_tdm_dspic33ck_sumprof_snapshot( out, clear_peak );
    tdm_all_rx_ie_restore( bak );

    return out->initialized;
}

#endif // NORA_TDM_SUMPROF


/*
 * Singleton load/status readers: report the block-timing-reference instance (SPI1).
 * Thin wrappers over the per-instance readers; behaviour is unchanged from before
 * the per-instance API was added.
 */
bool nora_spi_i2s_tdm_get_load( nora_spi_i2s_tdm_load_t* monitor, bool clear_peak )
{
    return nora_spi_i2s_tdm_inst_get_load( &s_spi_legs[TDM_SPI_LEG_BLOCK_REF], monitor, clear_peak );
}

bool nora_spi_i2s_tdm_get_status( nora_spi_i2s_tdm_status_t* status, bool clear_peak )
{
    return nora_spi_i2s_tdm_inst_get_status( &s_spi_legs[TDM_SPI_LEG_BLOCK_REF], status, clear_peak );
}


//===========================================================
// DMA INTERRUPT VECTORS  --  IMPORTANT: these ARE the IVT entries the CPU jumps to.
//
// The HAL ships its own DMA interrupt vectors so the transport is turnkey: link the HAL
// and the _DMAnInterrupt slots are filled, the DMA channels are already armed by
// start(), and the integrator only registers a per-instance block callback. (Previously
// these lived in a separate optional TU nora_spi_i2s_tdm_irq.c + a forwarding
// worker; folded back here -- no cross-TU hop -- since the toolchain/IVT coupling is
// confined to this one section.)
//
// One EXPLICIT RX vector per leg. The RX channel's vector runs that instance's block ISR
// (tdm_rx_block with THIS leg + its RX/TX channels + its slots*blk half size, ALL
// compile-time constants so the DMA register access / pointer math fold). The TX channel
// is INTENTIONALLY interrupt-less (hw.c enables the CPU IRQ on the RX channel only): TX is
// fire-and-forget ping-pong with auto-reload, so the RX completion alone defines the block
// boundary -- there is no TX ISR.
//
// The vector names are LITERAL and greppable (they were token-pasted before). At the
// default channel mapping (conf.h) this defines:
//     _DMA0Interrupt   (leg SPI1 RX, DMA0)
//     _DMA2Interrupt   (leg SPI2 RX, DMA2, when NORA_TDM_USE_SPI2)
// A conf.h remap (e.g. SPI1 -> DMA6) no longer moves the vector by itself: it trips the
// per-vector binding assert below, and the name must be changed here to match. IVT slots
// are chip-wide exclusive; a genuine clash surfaces as a duplicate-_DMAnInterrupt link
// error.
//
// IVT slots are chip-wide exclusive. If another subsystem must own one of these
// channels, REMAP the SPI to a free channel in conf.h (preferred -- the vector follows
// by token-paste); a genuine clash surfaces as a duplicate-_DMAnInterrupt link error.
//===========================================================

// One EXPLICIT RX interrupt vector per leg (was X-macro-generated by token-pasting the
// channel number into the vector name). tdm_rx_block is static inline (same TU) so the
// constant RX/TX channels + half size fold into the register access -- NO runtime
// channel->leg lookup. (tx) is still passed -- tdm_rx_block reads the TX channel's DMA
// address to pick the writable TX half -- but the TX channel itself raises no interrupt
// (interrupt-less; see above).
//
// The vector NAME encodes the DMA channel, and the name is now a LITERAL rather than
// pasted from the macro, so the two can disagree. Each vector is therefore bound to its
// conf.h RX-DMA channel by a compile-time assert immediately above it: remap a channel in
// conf.h and the build FAILS there until the vector name is updated to match. (Under the
// former token-paste the name followed the macro and could not disagree; the assert buys
// that guarantee back.)
//
// Compiled only when the HAL owns the IVT (NORA_TDM_DEFINE_DMA_VECTORS=1, default).
// With =0 the integrator owns the vectors and calls nora_spi_i2s_tdm_inst_rx_isr()
// (below) from their own _DMA<rx>Interrupt instead.
#if NORA_TDM_DEFINE_DMA_VECTORS
TDM_COMPILEASSERT( (NORA_TDM_SPI1_RX_DMA) == 0 );   /* leg SPI1 RX -> _DMA0Interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI1],
                  NORA_TDM_SPI1_RX_DMA, NORA_TDM_SPI1_TX_DMA,
                  TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) );
}
#if NORA_TDM_USE_SPI2
TDM_COMPILEASSERT( (NORA_TDM_SPI2_RX_DMA) == 2 );   /* leg SPI2 RX -> _DMA2Interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _DMA2Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI2],
                  NORA_TDM_SPI2_RX_DMA, NORA_TDM_SPI2_TX_DMA,
                  TDM_LEG_HALF_SLOTS(NORA_TDM_SLOTS_PER_FS, NORA_TDM_BLOCK_FRAMES) );
}
#endif // NORA_TDM_USE_SPI2
#endif // NORA_TDM_DEFINE_DMA_VECTORS

/*
 * Public RX-block ISR entry for ONE instance (vector-ownership opt-out path).
 *
 * Runs the same block work as the HAL's own generated vector, but is a plain (non-
 * interrupt) function the integrator calls from their OWN _DMA<rx>Interrupt when
 * NORA_TDM_DEFINE_DMA_VECTORS=0. Channels + half size come from the leg at runtime
 * (the generated turnkey vectors fold them as constants; this dispatch trades that for
 * IVT ownership). NULL inst is ignored. Call it for the instance's RX channel only --
 * TX is interrupt-less.
 */
void nora_spi_i2s_tdm_inst_rx_isr( nora_spi_i2s_tdm_inst_t* inst )
{
    if( inst == NULL )
    {
        return;
    }
    tdm_rx_block( inst, inst->rx_dma_ch, inst->tx_dma_ch,
                  (uint32_t)inst->geom_slots_per_fs * (uint32_t)inst->geom_block_frames );
}


//===========================================================
// Local Function
//===========================================================

/*
 * Clear one instance's DMA ping-pong buffers (RX + TX).
 *
 * This covers transport buffers only. Application DSP work buffers live in
 * audio_app.c and are deliberately outside the HAL's ownership boundary.
 */
static void tdm_inst_clear_buffers( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    tdm_zero_memory( leg->tx_buffer, leg->buffer_slot_count * sizeof(leg->tx_buffer[0]) );
    tdm_zero_memory( leg->rx_buffer, leg->buffer_slot_count * sizeof(leg->rx_buffer[0]) );
}


/*
 * Tiny local memset(0) helper.
 *
 * Keeps this embedded HAL independent of the C library's memset implementation
 * and safely accepts NULL for callers that are already walking descriptor tables.
 */
static void tdm_zero_memory(void *ptr, size_t bytes)
{
    uint8_t *p = (uint8_t *)ptr;

    if( p == NULL )
    {
        return;
    }

    while( bytes > 0u )
    {
        *p = 0u;
        p++;
        bytes--;
    }
}


/*
 * Stop one instance's DMA activity.
 *
 * Its RX/TX interrupts are masked first so the ISR cannot refill buffers while the
 * channels are being disabled. The DMA controller global ON state is intentionally
 * untouched because other subsystems may be using DMA.
 */
static void tdm_inst_soft_stop_dma( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    nora_dma_irq_enable( leg->rx_dma_ch, false );
    nora_dma_irq_enable( leg->tx_dma_ch, false );
    (void)nora_dma_channel_enable( leg->rx_dma_ch, false );
    (void)nora_dma_channel_enable( leg->tx_dma_ch, false );
}


/*
 * Clear one instance's pending DMA status and CPU interrupt flags.
 *
 * Status is cleared before IRQ flags so stale HALF/DONE conditions cannot
 * immediately retrigger on the next start.
 */
static void tdm_inst_clear_dma_flags( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    nora_dma_clear_status( leg->rx_dma_ch );
    nora_dma_clear_status( leg->tx_dma_ch );
    nora_dma_clear_irq_flag( leg->rx_dma_ch );
    nora_dma_clear_irq_flag( leg->tx_dma_ch );
}


/*
 * Validate one private SPI leg descriptor.
 *
 * Checks only descriptor-local invariants: a known SPI instance, distinct RX/TX
 * DMA channels, non-NULL buffers, and a non-zero buffer length. Cross-leg
 * singleton rules are enforced by tdm_stream_topology_is_valid().
 */
static bool tdm_spi_leg_is_valid( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return false;
    }
    if( (unsigned)leg->spi_inst >= (unsigned)TDM_SPI_INST_COUNT )
    {
        return false;
    }
    if( leg->rx_dma_ch == leg->tx_dma_ch )
    {
        return false;
    }
    if( ( leg->rx_buffer == NULL ) || ( leg->tx_buffer == NULL ) )
    {
        return false;
    }
    if( leg->buffer_slot_count == 0u )
    {
        return false;
    }
    return true;
}


/*
 * Validate the private stream topology.
 *
 * Generic over the instance count (1..TDM_SPI_INST_COUNT): every leg is descriptor-
 * valid, EXACTLY ONE leg is the block-timing REFERENCE, and that reference is the first
 * leg (TDM_SPI_LEG_BLOCK_REF -- the row is_running()/the singleton get_status()/is_active()
 * report). All legs use distinct physical SPIs and distinct DMA channels (cross-leg loop
 * below). The physical SPI and the RX/TX DMA channels per leg come from the instance
 * descriptor list (conf.h); NOTHING here is pinned to a specific SPI number or channel,
 * so adding an instance or remapping a leg's channels needs no edit to this check.
 *
 * The "exactly one reference, at leg 0" rule is KEPT deliberately: the stream is still
 * co-clocked and the singleton reporting API needs one primary leg. It is NOT a
 * clock-role constraint (clock role is per-leg, P3 Stage 1). It generalizes to truly
 * independent per-instance timing when per-instance clocks land (P3 Stage 2), not before.
 */
static bool tdm_stream_topology_is_valid( const tdm_stream_t *stream )
{
    uint8_t timing_ref_count = 0u;
    const tdm_spi_leg_t *timing_leg = NULL;

    if( ( stream == NULL ) ||
        ( stream->legs == NULL ) ||
        ( stream->leg_count == 0u ) ||
        ( stream->leg_count > (uint8_t)TDM_SPI_INST_COUNT ) )
    {
        return false;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        const tdm_spi_leg_t *leg = &stream->legs[i];

        if( !tdm_spi_leg_is_valid( leg ) )
        {
            return false;
        }
        if( leg->is_block_timing_master )
        {
            timing_ref_count++;
        }
    }
    if( timing_ref_count != 1u )
    {
        return false;
    }

    // The single block-timing reference must be the first leg (TDM_SPI_LEG_BLOCK_REF): its RX
    // ISR defines the block boundary and is what is_running()/get_status() report.
    timing_leg = tdm_stream_get_block_timing_master_leg( stream );
    if( ( timing_leg == NULL ) ||
        ( timing_leg != &stream->legs[TDM_SPI_LEG_BLOCK_REF] ) )
    {
        return false;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        for( uint8_t j = (uint8_t)(i + 1u); j < stream->leg_count; j++ )
        {
            const tdm_spi_leg_t *a = &stream->legs[i];
            const tdm_spi_leg_t *b = &stream->legs[j];

            if( a->spi_inst == b->spi_inst )
            {
                return false;
            }
            if( ( a->rx_dma_ch == b->rx_dma_ch ) ||
                ( a->rx_dma_ch == b->tx_dma_ch ) ||
                ( a->tx_dma_ch == b->rx_dma_ch ) ||
                ( a->tx_dma_ch == b->tx_dma_ch ) )
            {
                return false;
            }
        }
    }

    return true;
}


/*
 * Find the single block-timing-reference SPI leg.
 *
 * Returns NULL if the table has no reference or more than one. The timing
 * leg is the row whose RX DMA interrupt defines block completion and invokes the
 * application's block callback.
 */
static const tdm_spi_leg_t *tdm_stream_get_block_timing_master_leg( const tdm_stream_t *stream )
{
    const tdm_spi_leg_t *timing_leg = NULL;

    if( ( stream == NULL ) || ( stream->legs == NULL ) )
    {
        return NULL;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        if( stream->legs[i].is_block_timing_master )
        {
            if( timing_leg != NULL )
            {
                return NULL;
            }
            timing_leg = &stream->legs[i];
        }
    }

    return timing_leg;
}


/*
 * Build the hardware-applied config for one SPI leg.
 *
 * Just a validated copy of the leg's stored config -- each leg carries its OWN clock
 * role (master/slave), so there is no stream-wide role override here. A follower is
 * SLAVE because the integrator configured it SLAVE (it rides the shared clock), not
 * because the HAL forces it; an instance on an independent clock can be its own master.
 */
static bool tdm_spi_leg_get_effective_config( const tdm_spi_leg_t *leg,
                                              nora_spi_i2s_tdm_config_t *effective_cfg )
{
    if( ( effective_cfg == NULL ) || !tdm_spi_leg_is_valid( leg ) || !leg->config_valid )
    {
        return false;
    }

    *effective_cfg = leg->config;
    return true;
}








// Deadline-miss, ISR load/time, and the debug scope-GPIO/printf instrumentation
// that used to live here (local_dma_debug_check / _start / _end) now live in the
// separated diagnostics module (nora_spi_i2s_tdm_diag.*), reached through
// nora_spi_i2s_tdm_diag_check_deadline / _isr_begin / _isr_end.









/*
 * Resolve the RX ping-pong half completed by the DMA status snapshot.
 *
 * Output is NULL when the status does not identify a usable half or when inputs
 * are invalid. The returned pointer is into the HAL-owned RX buffer and is passed
 * read-only to the application block callback.
 */
static inline void tdm_get_src_ptr( nora_dma_status_t    dma_stat,
                                      const nora_tdm_slot_t* const pRxDat,
                                      uint32_t             half_pos,
                                      const nora_tdm_slot_t**      src_pptr )
{
    if( src_pptr == NULL )
    {
        return;
    }
    *src_pptr = NULL;

    if( pRxDat == NULL )
    {
        return;
    }

    // half_pos = this instance's ping/pong half size in SLOTS (slots * blk).
    switch( nora_dma_half_from_status_hot( dma_stat ) )
    {
    case NORA_DMA_HALF_FIRST:
        // SW can use Ping(A) side buffer.
        //////////////////////////////////////
        *src_pptr  = &pRxDat[ DMA_BUF_PING_POS ];
        break;

    case NORA_DMA_HALF_SECOND:
        // SW can use Pong(B) side buffer.
        //////////////////////////////////////
        *src_pptr  = &pRxDat[ half_pos ];
        break;

    default:
        break;
    }
}

/*
 * Resolve the TX ping-pong half that software may safely fill next.
 *
 * The active DMA source address tells which half DMA is currently reading; the
 * helper returns the opposite half. Output is NULL when inputs are invalid.
 */
static inline void tdm_get_dest_ptr( uint32_t       dma_tx_addr,
                                       nora_tdm_slot_t* const pTxDat,
                                       uint32_t       half_pos,
                                       nora_tdm_slot_t**      dest_pptr )
{
    if( dest_pptr == NULL )
    {
        return;
    }
    *dest_pptr = NULL;

    if( pTxDat == NULL )
    {
        return;
    }

    // half_pos = this instance's ping/pong half size in SLOTS (slots * blk). The buffer
    // is [base, end) = pTxDat[0 .. 2*half_pos); the pong half starts at pTxDat[half_pos].
    // GUARD: the active TX-DMA source address must actually lie inside this buffer. At a
    // reload boundary, just after stop, on the first block, or in a fault, DMAxSRC can
    // hold an out-of-range value -- a bare ">= mid" test would then misclassify the half
    // and hand back a pointer to the half DMA is transmitting. Out of range -> NULL (the
    // caller NULL-checks and skips this block's fill).
    const uintptr_t base = (uintptr_t)&pTxDat[ DMA_BUF_PING_POS ];
    const uintptr_t mid  = (uintptr_t)&pTxDat[ half_pos ];
    const uintptr_t end  = (uintptr_t)&pTxDat[ 2u * half_pos ];
    const uintptr_t addr = (uintptr_t)dma_tx_addr;   // DMAxSRC snapshot as an address

    if( ( addr < base ) || ( addr >= end ) )
    {
        return;   // *dest_pptr stays NULL
    }

    if( addr >= mid )
    {
        // DMA is reading the Pong half -> SW fills Ping
        *dest_pptr = (nora_tdm_slot_t*)&pTxDat[ DMA_BUF_PING_POS ];
    }
    else
    {
        // DMA is reading the Ping half -> SW fills Pong
        *dest_pptr = (nora_tdm_slot_t*)&pTxDat[ half_pos ];
    }
}


/*
 * Validate the configuration envelope for ONE instance.
 *
 * The HAL accepts only what it can actually program/size: 32-bit words; geometry
 * (slots_per_fs / block_frames) that MATCHES this leg's compile-time buffer geometry
 * (its Rx_<name>/Tx_<name> are sized for exactly that, per the instance list); an
 * explicit role; and a framing the FRMCNT path supports -- I2S with 2 slots, or TDM
 * with 4/8/16 slots. This keeps configure() from programming untested framing or a
 * geometry the static buffers were not sized for.
 *
 * It also rejects the ONE combination the HAL would answer with a lying success:
 * I2S + MASTER + FS_50PCT, whose generated FS is 25% duty on a 16-bit wire. See the
 * fs_shape block at the end of this function.
 *
 * TDM32 (32 slots/FS) is unsupported. The serial wire
 * word is 16 bits, so a 32-bit slot is two wire words and FS cadence is measured in wire
 * words: 32 slots needs FRMCNT to encode 64, and the field stops at 32 (log2 -> 5). This
 * is a real capability loss, not a tidy-up -- it is stated here rather than left to be
 * discovered as a configure() that returns false for no visible reason. 16 slots x 32 bit
 * = 32 wire words is the widest TDM this part can frame. Recovering TDM32 would mean
 * 16-bit slots (word_bits = 16, one wire word per slot), which the rest of the HAL does
 * not currently support.
 */
static bool tdm_config_is_supported( const tdm_spi_leg_t* leg, const nora_spi_i2s_tdm_config_t* cfg )
{
    if( ( cfg == NULL ) || ( leg == NULL ) )  return false;

    if( cfg->word_bits != 32u )         return false;
    // Geometry must match THIS leg's compile-time (statically allocated) geometry.
    if( cfg->slots_per_fs != leg->geom_slots_per_fs ) return false;
    if( cfg->block_frames != leg->geom_block_frames ) return false;

    // role must be an explicit SLAVE or MASTER -- otherwise a garbage value would be
    // silently treated as SLAVE everywhere (role == MASTER ? ... : SLAVE).
    if( ( cfg->clock_role != NORA_SPI_I2S_TDM_CLOCK_SLAVE ) &&
        ( cfg->clock_role != NORA_SPI_I2S_TDM_CLOCK_MASTER ) )
    {
        return false;
    }

    if( cfg->format == NORA_SPI_I2S_TDM_FORMAT_I2S )
    {
        if( cfg->slots_per_fs != 2u )   return false;        // I2S = 2 slots (L/R)
    }
    else if( cfg->format == NORA_SPI_I2S_TDM_FORMAT_TDM )
    {
        // TDM: FS cadence is FRMCNT-encoded in WIRE WORDS (2 per 32-bit slot), and FRMCNT
        // tops out at 32 wire words -- so 4/8/16 slots, NOT 32. See the header comment.
        if( ( cfg->slots_per_fs != 4u ) && ( cfg->slots_per_fs != 8u ) &&
            ( cfg->slots_per_fs != 16u ) )
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    // fs_shape must be a known value -- otherwise a garbage value would be silently
    // treated as FS_PULSE by hw_apply_config (shape == FS_50PCT ? ... : ...).
    if( ( cfg->fs_shape != NORA_SPI_I2S_TDM_FS_PULSE ) &&
        ( cfg->fs_shape != NORA_SPI_I2S_TDM_FS_50PCT ) )
    {
        return false;
    }

    // I2S + MASTER + FS_50PCT is REJECTED, and the reject is the point: this leg is the one
    // combination where the HAL would GENERATE a frame sync whose duty does not match the
    // shape the caller asked for. The wire word is 16 bits, so the
    // FRMSYPW=1 pulse hw_apply_config() programs for I2S is 16 BCLK out of a 64-BCLK
    // (2 x 32-bit) frame = 25%, not 50%. Returning true here would hand back a SUCCESS for a
    // 50%-duty request and then emit 25% -- a lying success is worse than a refusal, because
    // nothing downstream can detect it. The fix is the same CLC half-frame-marker route the
    // TDM master path already uses; until that exists and has been seen on a scope, this
    // combination is unsupported rather than silently wrong.
    //
    // Everything else stays accepted, deliberately:
    //   - I2S + SLAVE + FS_50PCT: FS is an INPUT, so the HAL generates no waveform and makes
    //     no duty claim. FRMSYPW still describes how the incoming FS is read, which is why
    //     fs_shape remains a compared framing field within a sync domain.
    //   - TDM (any role) + FS_50PCT: the master gets a real ~50% FS from CLC1 (measured); a
    //     TDM slave receives FS as an input and hw_apply_config treats it as normal framing.
    if( ( cfg->format     == NORA_SPI_I2S_TDM_FORMAT_I2S ) &&
        ( cfg->clock_role == NORA_SPI_I2S_TDM_CLOCK_MASTER ) &&
        ( cfg->fs_shape   == NORA_SPI_I2S_TDM_FS_50PCT ) )
    {
        return false;
    }

    // Note: sample rate is NOT part of the transport envelope -- the core is
    // rate-agnostic (runs at the configured BRG / external clock). The product's
    // supported-rate policy lives in the app layer (APP_SAMPLE_RATE_IS_SUPPORTED), not here.
    return true;
}


/*
 * Read-and-clear / restore the CPU interrupt enable of an instance's RX DMA channel.
 *
 * Used to bracket non-atomic updates against the instance's RX-block ISR. Channel-
 * generic (via the DMA HAL), so it follows whatever DMA channel an instance is mapped
 * to in conf.h -- no per-channel hardcode. Returns the prior IE state; restore re-arms
 * only if it was enabled (so masking before start() leaves the IE off).
 */
static inline bool tdm_rx_ie_disable( nora_dma_channel_t rx_dma_ch )
{
    return nora_dma_irq_disable_save( rx_dma_ch );
}

static inline void tdm_rx_ie_restore( nora_dma_channel_t rx_dma_ch, bool was_enabled )
{
    nora_dma_irq_restore( rx_dma_ch, was_enabled );
}


/*
 * One SPI instance's RX-block ISR body (generic).
 *
 * Snapshots the instance's RX DMA status, maps the just-completed RX half and the
 * writable TX half of THIS instance, then invokes THIS instance's callback with
 * (src, dst, user). Called from the generated per-instance RX vector (the
 * _DMA<rx>Interrupt section above), which passes this instance's leg + channels.
 *
 * The generated-vector callers pass compile-time constants.  `always_inline`
 * (on both declaration and definition) is required here: -Os previously chose
 * to outline this body, which silently turned those values back into runtime
 * arguments and retained the generic DMA switch path.  The public
 * inst_rx_isr() fallback may still use runtime values; it is an integrator-owned
 * vector path, not the EV88G73A's performance-critical default.
 */
static inline __attribute__((always_inline)) void
tdm_rx_block( tdm_spi_leg_t* inst, nora_dma_channel_t rx_ch, nora_dma_channel_t tx_ch, uint32_t half_pos )
{
          nora_dma_status_t  dma_stat;
    const nora_tdm_slot_t*  src_ptr = NULL;
          nora_tdm_slot_t*  dst_ptr = NULL;
          bool      time_this_block;

    // Engine-wide TDMsum union hook. Bracketing the whole body, so SPI1 and SPI2 add into one
    // common-window occupancy. Gated on the same high-res-timer availability as the per-leg
    // monitor; sum_meas is stable across this ISR so enter/exit stay balanced on every return
    // path.
    //
    // NOT gated on time_this_block: the per-leg monitor is sampled (1 block in 2^N, see
    // NORA_TDM_ISR_TIMING_SAMPLE_LOG2) and a UNION occupancy cannot be sampled -- a window
    // whose peak fell in an unmeasured block would read low. So this pays two timer reads on
    // EVERY block, which is exactly what NORA_TDM_SUMPROF=0 buys back (hook, timer read and
    // all): the hooks cost ISR cycles whether or not anyone ever calls _tdmsum_get().
#if NORA_TDM_SUMPROF
    const bool sum_meas = nora_high_res_timer_is_initialized();
    if( sum_meas )
    {
        nora_spi_i2s_tdm_dspic33ck_sumprof_enter( nora_high_res_timer_get_count() );
    }
#endif

    // Safety diagnostics below stay exact on EVERY block.  Only the costly
    // timer-based execution-time profiler is sampled (default 1/16) because
    // it needs two coherent 32-bit timer reads plus volatile 32-bit min/max
    // bookkeeping.  The resulting TDM max is explicitly a sampled peak.
    time_this_block = nora_spi_i2s_tdm_diag_should_time_this_block( &inst->diag );
    if( time_this_block )
    {
        nora_spi_i2s_tdm_diag_isr_begin( &inst->diag );
    }

    dma_stat = nora_dma_isr_snapshot_hot( rx_ch );

    // Preserve the raw RX-DMA cause (esp. OVRUNIF) BEFORE the half/pointer early-returns
    // below: an overrun-only snapshot maps to no completed half and would otherwise be lost.
    // This tiny header-inline hook replaces an out-of-line call but keeps the same
    // status/counter semantics, including the overrun-only early-return case.
    nora_spi_i2s_tdm_diag_note_dma_status_hot( &inst->diag, dma_stat );

    // Stream-health check; diagnostic print is debug-only. Each instance counts its
    // own deadline misses in its own diag (no shared/master counter).  Avoid the
    // former out-of-line helper call on the clean path, but retain that helper for
    // the rare fault so ENA_TDM_DBG still prints the same DMA/channel evidence.
    if( nora_dma_status_has_half_done_conflict_hot( dma_stat ) )
    {
        nora_spi_i2s_tdm_diag_check_deadline( &inst->diag, rx_ch, dma_stat );
    }

    // Map this instance's completed RX half (callback input) and the TX half it may
    // fill (callback output). Each instance handles only its own RX/TX -- no dst_b.
    tdm_get_src_ptr( dma_stat, inst->rx_buffer, half_pos, &src_ptr );
    if( src_ptr == NULL )
    {
        if( time_this_block )
        {
            nora_spi_i2s_tdm_diag_isr_end( &inst->diag );
        }
#if NORA_TDM_SUMPROF
        if( sum_meas )
        {
            nora_spi_i2s_tdm_dspic33ck_sumprof_exit( nora_high_res_timer_get_count() );
        }
#endif
        return;
    }
    tdm_get_dest_ptr( nora_dma_read_src_hot( tx_ch ), inst->tx_buffer, half_pos, &dst_ptr );
    if( dst_ptr == NULL )
    {
        // TX-DMA source is out of the buffer envelope (reload boundary / just-stopped /
        // first block / fault): there is no writable TX half this block. Skip rather than
        // hand the callback a NULL dst -- the public contract is that dst is always valid
        // when block_cb runs. (Mirrors the src_ptr guard above.)
        if( time_this_block )
        {
            nora_spi_i2s_tdm_diag_isr_end( &inst->diag );
        }
#if NORA_TDM_SUMPROF
        if( sum_meas )
        {
            nora_spi_i2s_tdm_dspic33ck_sumprof_exit( nora_high_res_timer_get_count() );
        }
#endif
        return;
    }

    nora_spi_i2s_tdm_diag_note_block_hot( &inst->diag ); // exact completed-block count

    // Sample + ack this instance's framed-transport health once per completed block. Passing
    // it every block (even a clean one) is what lets frmerr_consecutive_blocks reset, so a
    // one-off glitch does not read as a permanent frame slip.  `spi_statl` was validated and
    // cached before inst_start() enabled DMA/SPI, so this is a header-inline volatile
    // read/W0C-ack sequence -- not an enum/table lookup plus out-of-line function call.
    // The helper deliberately preserves every-block observation and the W0C-safe semantics;
    // this is a function-boundary optimization, not a diagnostic-quality trade-off.
    nora_spi_i2s_tdm_diag_note_errflags_hot( &inst->diag,
        nora_spi_i2s_tdm_hw_sample_ack_errflags_hot( inst->spi_statl ) );

    // Deliver the completed block through this instance's registered callback. The
    // callee owns its DSP work buffers; the driver passes only this instance's
    // selected RX/TX ping-pong halves (both guaranteed non-NULL here). No callback ->
    // no app/DSP path: start() zeroes the TX half so a fresh start is silent until a
    // callback fills it (clearing the callback mid-stream leaves the last TX data looping).
    if( inst->block_cb != NULL )
    {
        inst->block_cb( src_ptr, dst_ptr, inst->block_user );
    }

    if( time_this_block )
    {
        nora_spi_i2s_tdm_diag_isr_end( &inst->diag );
    }
#if NORA_TDM_SUMPROF
    if( sum_meas )
    {
        nora_spi_i2s_tdm_dspic33ck_sumprof_exit( nora_high_res_timer_get_count() );
    }
#endif
}


// The demo/application audio path (local_copy_to_CODEC,
// local_drc_df2t_path, local_filter_cascade_chm) and the optional PWM audio
// output live in the demo app (audio_app.c). The HAL core does NOT
// call them: each instance's RX-block handler delivers its block to the registered
// block callback only (no app fallback). The HAL core owns the DMA ping-pong buffers;
// the demo app owns its f_*_Data DSP work buffers.
