#ifndef NORA_SPI_I2S_TDM_DIAG_H
#define NORA_SPI_I2S_TDM_DIAG_H

//===========================================================
// nora_spi_i2s_tdm_diag.h (contract) + nora_spi_i2s_tdm_dspic33ck_diag.c (CK backend)
// = the SPI/I2S/TDM transport's DIAGNOSTICS, kept deliberately SEPARATE from the
// transport core. It owns the per-stream
// health counters (completed block count, deadline-miss count) and the per-ISR
// load/time monitor, plus the only debug-build dependencies (high-res timer for
// the load monitor, and -- under ENA_TDM_DBG -- printf + scope GPIO). The
// transport core holds one nora_spi_i2s_tdm_diag_t per block-completion ISR
// and updates it ONLY through the functions below; it does not read the fields
// in the hot path. This separation keeps the transport context limited to what is
// required to configure/start/transfer/stop, and lets each physical SPI instance
// own an independent diag once the engine moves to per-instance ISRs.
//
// CONCURRENCY: the counters are updated from the block-completion ISR. The 32-bit
// reads in *_get_load()/*_read_counts() are NOT atomic on this 16-bit core, so the
// CALLER must mask the updating ISR's CPU interrupt around them (the transport core
// already brackets the public readers with the DMA IE mask). The functions here do
// no masking themselves.
//===========================================================

#include <stdint.h>
#include <stdbool.h>
#include "nora_spi_i2s_tdm.h"   // nora_spi_i2s_tdm_load_t (public load-monitor type)
#include "nora_dma.h"            // nora_dma_status_t + the portable status predicates
// Nothing family-specific is included here on purpose. The per-block ISR hooks are inline
// and need the CK DMA status predicates plus the CK SPIxSTATL bit masks, so they live in
// the backend sibling nora_spi_i2s_tdm_dspic33ck_diag_fast.h. A header carrying the
// neutral nora_ name must not pull one family's register layout in behind it.


//===========================================================
// One block-completion ISR's diagnostics. block_count / block_deadline_miss_count
// are software/real-time stream health (NOT SPI HW over/underrun). The isr_*
// counters are raw high-res-timer ticks of ISR execution time; *_get_load()
// converts them to the public monitor (counts + 0.1us units).
//===========================================================
typedef struct {
    volatile uint32_t block_count;               // completed blocks since reset()
    volatile uint32_t block_deadline_miss_count; // HALF+DONE conflicts on this instance since reset()
    // ---- SPI framed-transport health (SPIxSTATL) + RX-DMA overrun ----
    // These are HARDWARE over/underrun and frame-slip observations, DISTINCT from the
    // software/real-time block_deadline_miss_count above. Each err_*_block_count counts
    // RX BLOCKS in which the flag was observed, not raw events.
    volatile uint32_t rx_dma_overrun_count;      // RX ISR snapshots with DMAINTn.OVRUNIF since reset()
    volatile uint32_t rx_dma_other_irq_count;    // RX IRQ snapshots with neither HALF nor DONE since reset()
    // DMAINTn is a 16-bit register on dsPIC33CK.  Keeping the internal last
    // snapshot 16-bit avoids a needless 32-bit volatile store on every normal
    // RX block; the public status API still widens it to uint32_t for source
    // compatibility with its existing field.
    volatile uint16_t rx_dma_last_status;        // raw DMAINTn from the most recent RX IRQ
    volatile uint32_t err_rov_block_count;       // RX blocks where SPIROV was observed, since reset()
    volatile uint32_t err_tur_block_count;       // RX blocks where SPITUR was observed set, since reset()
    volatile uint32_t err_frm_block_count;       // RX blocks where FRMERR was observed, since reset()
    volatile uint32_t frmerr_consecutive_blocks; // consecutive RX blocks with FRMERR observed (0 on a clean block)
    volatile uint32_t isr_start_count;           // timer count at the current ISR entry
    volatile uint32_t isr_last_count;            // ticks of the last completed ISR
    volatile uint32_t isr_min_count;             // min ticks since the last clear_peak
    volatile uint32_t isr_max_count;             // max ticks since the last clear_peak
    // Number of TIMED SAMPLES, not necessarily number of RX blocks.  See the
    // sample countdown below and NORA_TDM_ISR_TIMING_SAMPLE_LOG2.
    volatile uint32_t isr_event_count;
    volatile bool     isr_measure_active;        // current ISR has a valid start sample
    // A deliberately small, 16-bit-core-friendly prescaler.  The hot path
    // decrements this once per RX block; only when it reaches zero does it pay
    // for timer reads and volatile 32-bit min/max bookkeeping.
    volatile uint8_t  isr_timing_sample_countdown;
} nora_spi_i2s_tdm_diag_t;


//===========================================================
// The per-block ISR fast path is NOT here. The `_hot` hooks
// (_should_time_this_block, _note_dma_status_hot, _note_block_hot,
// _note_errflags_hot) are backend-private inlines and live in
// nora_spi_i2s_tdm_dspic33ck_diag_fast.h, which the CK backend translation unit
// includes on top of this header. The out-of-line twins below are the portable
// entry points and remain valid callers for anything outside a hot path.
//===========================================================

// Reset every counter (isr_min_count seeded to UINT32_MAX). Called by start().
void nora_spi_i2s_tdm_diag_reset( nora_spi_i2s_tdm_diag_t* d );

// Begin/end one SELECTED ISR timing sample.  The transport calls these only when
// diag_should_time_this_block() says so.  begin() snapshots the entry tick (only
// when the high-res timer HAL is initialized) and, under ENA_TDM_DBG, raises the
// scope GPIO; end() records sampled last/min/max/event and lowers the scope GPIO.
void nora_spi_i2s_tdm_diag_isr_begin( nora_spi_i2s_tdm_diag_t* d );
void nora_spi_i2s_tdm_diag_isr_end( nora_spi_i2s_tdm_diag_t* d );

// Count one completed block for THIS instance's RX-block ISR (read via
// *_read_counts() / get_status()). Each instance keeps its own block_count.
void nora_spi_i2s_tdm_diag_note_block( nora_spi_i2s_tdm_diag_t* d );

// Update deadline diagnostics from THIS instance's DMA status snapshot. A HALF+DONE
// conflict means this instance's RX-block ISR fell a full block behind, so it counts
// one deadline miss in its OWN diag -- each instance keeps its own block/miss counts
// (per-instance diagnostics; there is no shared "block master" counter). dma_x is used
// only for the debug print label.
void nora_spi_i2s_tdm_diag_check_deadline( nora_spi_i2s_tdm_diag_t* d,
                                                nora_dma_channel_t dma_x,
                                                nora_dma_status_t  dma_stat );

// Preserve one raw RX-DMA IRQ cause before HALF/DONE resolution. DMAINTn.OVRUNIF is the
// primary transport-stall signal (a trigger arrived while the channel still had a pending
// request); an overrun-only snapshot cannot map to a completed ping-pong half, so calling
// this BEFORE the core's half/pointer early-returns keeps the root-cause evidence. MUST be
// called once per RX-block ISR, right after the DMA status snapshot.
void nora_spi_i2s_tdm_diag_note_dma_status( nora_spi_i2s_tdm_diag_t* d,
                                                 nora_dma_status_t dma_stat );

// Fold one RX-block's SPIxSTATL flag observation into this instance's diagnostics. MUST be
// called once per completed block (even when flags==0) so frmerr_consecutive_blocks resets on
// a clean block. `flags` is the nora_spi_i2s_tdm_hw_sample_ack_errflags_hot() mask. Each
// counter counts RX BLOCKS in which its bit was observed, not raw events. When FRMERR is
// absent the consecutive run is reset to zero; the other counters are unchanged when flags==0.
void nora_spi_i2s_tdm_diag_note_errflags( nora_spi_i2s_tdm_diag_t* d, uint16_t flags );

// Snapshot the load monitor into the public struct (and clear min/max/event when
// clear_peak). Returns false (and zeroes the monitor) until at least one timed
// sample exists and the high-res timer HAL is initialized. The caller MUST mask the
// updating ISR around this call (see CONCURRENCY note above).
/*
 * Fill the us10 fields of a snapshot taken by _get_load(). MUST be called with
 * interrupts restored -- the conversion is three 64-bit divides and belongs
 * outside the RX-DMA-IE mask, which otherwise grows long enough to delay the
 * block ISR and skew the timing being measured. _get_load() leaves the us10
 * fields zeroed for this reason.
 */
void nora_spi_i2s_tdm_diag_load_convert( nora_spi_i2s_tdm_load_t* monitor );

bool nora_spi_i2s_tdm_diag_get_load( nora_spi_i2s_tdm_diag_t* d,
                                          nora_spi_i2s_tdm_load_t*  monitor,
                                          bool                           clear_peak );

// Read the two block counters. The caller MUST mask the updating ISR around this
// call. Either output pointer may be NULL.
void nora_spi_i2s_tdm_diag_read_counts( const nora_spi_i2s_tdm_diag_t* d,
                                             uint32_t* block_count,
                                             uint32_t* block_deadline_miss_count );


#endif // NORA_SPI_I2S_TDM_DIAG_H
