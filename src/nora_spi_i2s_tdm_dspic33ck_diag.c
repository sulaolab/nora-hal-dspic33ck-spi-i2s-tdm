
//===========================================================
// INCLUDES
//===========================================================
#include "nora_spi_i2s_tdm_diag.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>                   // NULL
#include "nora_high_res_timer.h" // nora_high_res_timer_* (ISR load/time monitor; runtime-gated via is_initialized())
#include "nora_dma.h"            // nora_dma_status_has_half_done_conflict(), NORA_DMA_STAT_*
#include "nora_spi_i2s_tdm_dspic33ck_reg.h" // NORA_SPI_I2S_TDM_STATL_SPIROV/SPITUR/FRMERR (frame-slip flags)
// The TDMsum profiler's mutable state and its ISR fast path live in the backend fast header;
// this translation unit supplies that state and the foreground configure/reset/snapshot half.
#include "nora_spi_i2s_tdm_dspic33ck_diag_fast.h"


//===========================================================
// Definition
//===========================================================

// ---- Debug / diagnostics master switch ----
// Default OFF: the diagnostics compile with no printf dependency and no debug GPIO
// toggles. Define ENA_TDM_DBG (here or via the build configuration) to restore the
// printf / scope-GPIO debug behavior. The load/time monitor itself is NOT gated:
// its accumulators are always captured so the load monitor works in production
// when the high-resolution timer HAL has been initialized.
//#define ENA_TDM_DBG

#if defined(ENA_TDM_DBG)
  #include <stdio.h>                 // printf (debug build only)
  #include "nora_tick_timer.h"  // timestamp for the debug trap print
  #include "board_dbg_pins.h"        // BOARD_DBG_PIN_* scope pins
  #define TDM_DBG_PRINTF(...)   printf(__VA_ARGS__)
#else
  #define TDM_DBG_PRINTF(...)   ((void)0)
#endif //defined(ENA_TDM_DBG)


//===========================================================
// Global Function
//===========================================================

/*
 * Reset every diagnostic counter.
 *
 * isr_min_count is seeded to UINT32_MAX so the first timed ISR sample becomes the
 * minimum. start() calls this so each stream run reports fresh statistics.
 */
void nora_spi_i2s_tdm_diag_reset( nora_spi_i2s_tdm_diag_t* d )
{
    if( d == NULL )
    {
        return;
    }

    d->block_count               = 0u;
    d->block_deadline_miss_count = 0u;
    d->rx_dma_overrun_count      = 0u;
    d->rx_dma_other_irq_count    = 0u;
    d->rx_dma_last_status        = 0u;
    d->err_rov_block_count       = 0u;
    d->err_tur_block_count       = 0u;
    d->err_frm_block_count       = 0u;
    d->frmerr_consecutive_blocks = 0u;
    d->isr_start_count           = 0u;
    d->isr_last_count            = 0u;
    d->isr_min_count             = 0xFFFFFFFFUL;
    d->isr_max_count             = 0u;
    d->isr_event_count           = 0u;
    d->isr_measure_active        = false;
    // Zero deliberately means "measure the first block".  The inline hot
    // helper then reloads period-1 and skips the configured number of blocks.
    d->isr_timing_sample_countdown = 0u;
}


/*
 * Begin one selected block-ISR timing sample.
 *
 * The load monitor is active only when the high-resolution timer HAL is already
 * initialized. Optional debug GPIO toggles are compiled in only for ENA_TDM_DBG.
 */
void nora_spi_i2s_tdm_diag_isr_begin( nora_spi_i2s_tdm_diag_t* d )
{
    if( d == NULL )
    {
        return;
    }

    // Load/time monitor: capture only when the high-resolution timer is live.
    d->isr_measure_active = nora_high_res_timer_is_initialized();
    if( d->isr_measure_active )
    {
        d->isr_start_count = nora_high_res_timer_get_count();
    }

#if defined(ENA_TDM_DBG)
    // debug-only scope GPIO: measuring the process time on a pin.
    (void)nora_gpio_toggle(BOARD_DBG_PIN_E4);
    (void)nora_gpio_set(BOARD_DBG_PIN_H0);
#endif //defined(ENA_TDM_DBG)
}


/*
 * End one selected block-ISR timing sample and update load statistics.
 *
 * Records last/min/max/event_count for *_get_load(). If the timer is unavailable,
 * measurement is simply abandoned for this ISR without affecting the audio path.
 */
void nora_spi_i2s_tdm_diag_isr_end( nora_spi_i2s_tdm_diag_t* d )
{
    // Load/time monitor: always accumulated (feeds *_get_load).
    uint32_t end_count;
    uint32_t diff_count;

    if( d == NULL )
    {
        return;
    }

    if( !d->isr_measure_active || !nora_high_res_timer_is_initialized() )
    {
        d->isr_measure_active = false;
#if defined(ENA_TDM_DBG)
        // debug-only scope GPIO: measuring the process time on a pin.
        (void)nora_gpio_clear(BOARD_DBG_PIN_H0);
#endif //defined(ENA_TDM_DBG)
        return;
    }

    d->isr_measure_active = false;

    end_count  = nora_high_res_timer_get_count();
    diff_count = end_count - d->isr_start_count;

    d->isr_last_count = diff_count;

    if( diff_count < d->isr_min_count )
    {
        d->isr_min_count = diff_count;
    }

    if( diff_count > d->isr_max_count )
    {
        d->isr_max_count = diff_count;
    }

    d->isr_event_count++;

#if defined(ENA_TDM_DBG)
    // debug-only scope GPIO: measuring the process time on a pin.
    (void)nora_gpio_clear(BOARD_DBG_PIN_H0);
#endif //defined(ENA_TDM_DBG)
}


/*
 * Count one completed block (read via *_read_counts() / get_status()).
 */
void nora_spi_i2s_tdm_diag_note_block( nora_spi_i2s_tdm_diag_t* d )
{
    if( d == NULL )
    {
        return;
    }
    d->block_count++;
}


/*
 * Update deadline-miss diagnostics from this instance's DMA status snapshot.
 *
 * HALF+DONE together means software missed a ping-pong service deadline for THIS
 * instance: its RX-block ISR fell a full block behind. Each instance keeps its own
 * block_deadline_miss_count, so the miss is counted in the passed-in diag.
 */
void nora_spi_i2s_tdm_diag_check_deadline( nora_spi_i2s_tdm_diag_t* d,
                                                nora_dma_channel_t dma_x,
                                                nora_dma_status_t  dma_stat )
{
    if( d == NULL )
    {
        return;
    }

    if( !nora_dma_status_has_half_done_conflict( dma_stat ) )
    {
        return;
    }

    // A HALF+DONE conflict means this instance's block ISR fell a full block behind.
    d->block_deadline_miss_count++;

    TDM_DBG_PRINTF(" dma_debug_check: dma=%d half/done conflict @%ld\n",
                   (int)dma_x,
                   nora_tick_timer_get_ms());
    (void)dma_x;
}


/*
 * Preserve one raw RX-DMA IRQ cause before HALF/DONE resolution.
 *
 * DMAINTn.OVRUNIF is the primary transport-stall signal: a trigger arrived while the
 * channel still had a pending request. An overrun-only snapshot cannot map to a
 * completed ping-pong half, so without this counter the core's early return would erase
 * the root-cause evidence. The raw last value also keeps unexpected DMA status bits
 * inspectable through the public status API.
 */
void nora_spi_i2s_tdm_diag_note_dma_status( nora_spi_i2s_tdm_diag_t* d,
                                                 nora_dma_status_t dma_stat )
{
    if( d == NULL )
    {
        return;
    }

    d->rx_dma_last_status = (uint16_t)dma_stat;
    if( nora_dma_status_has_overrun( dma_stat ) )
    {
        d->rx_dma_overrun_count++;
    }
    if( !nora_dma_status_has_completed_half( dma_stat ) )
    {
        d->rx_dma_other_irq_count++;
    }
}


/*
 * Fold one RX-block's SPIxSTATL flag observation into this instance's diagnostics.
 *
 * Each counter counts RX BLOCKS in which its bit was observed, not raw events. FRMERR is
 * a connector-glitch / frame bit-slip: the DMA keeps running so the transport looks healthy
 * but the audio is silently wrong. frmerr_consecutive_blocks gives the app a restart trigger
 * (detection here, recovery policy in the app). Reset the run on any clean block.
 */
void nora_spi_i2s_tdm_diag_note_errflags( nora_spi_i2s_tdm_diag_t* d, uint16_t flags )
{
    if( d == NULL )
    {
        return;
    }
    if( flags & NORA_SPI_I2S_TDM_STATL_SPIROV ) { d->err_rov_block_count++; }
    if( flags & NORA_SPI_I2S_TDM_STATL_SPITUR ) { d->err_tur_block_count++; }
    if( flags & NORA_SPI_I2S_TDM_STATL_FRMERR )
    {
        d->err_frm_block_count++;
        d->frmerr_consecutive_blocks++;
    }
    else if( d->frmerr_consecutive_blocks != 0u )
    {
        // Preserve the logical "clean block breaks the run" rule without
        // writing a 32-bit zero on every already-clean audio block.
        d->frmerr_consecutive_blocks = 0u;
    }
}


/*
 * Snapshot the block-ISR load monitor.
 *
 * The caller masks the updating ISR around this call. Returns false until at least
 * one timed event exists or if the high-resolution timer was not initialized. When
 * clear_peak is true, min/max/event accumulation starts fresh afterward.
 */
bool nora_spi_i2s_tdm_diag_get_load( nora_spi_i2s_tdm_diag_t* d,
                                          nora_spi_i2s_tdm_load_t*  monitor,
                                          bool                           clear_peak )
{
    bool     valid;
    uint32_t last_count;
    uint32_t min_count;
    uint32_t max_count;
    uint32_t event_count;

    if( ( d == NULL ) || ( monitor == NULL ) )
    {
        return false;
    }

    last_count  = d->isr_last_count;
    min_count   = d->isr_min_count;
    max_count   = d->isr_max_count;
    event_count = d->isr_event_count;

    if( clear_peak )
    {
        d->isr_min_count   = 0xFFFFFFFFUL;
        d->isr_max_count   = 0u;
        d->isr_event_count = 0u;
    }

    valid = (event_count != 0) && nora_high_res_timer_is_initialized();

    if( !valid )
    {
        last_count  = 0;
        min_count   = 0;
        max_count   = 0;
        event_count = 0;
    }

    monitor->last_count  = last_count;
    monitor->min_count   = min_count;
    monitor->max_count   = max_count;
    monitor->event_count = event_count;

    // us10 fields are deliberately NOT filled here. See
    // nora_spi_i2s_tdm_diag_load_convert(): the caller masks the RX DMA IE
    // around this function, and count_to_us_x10() is a 64-bit multiply plus a
    // 64-bit divide -- three of them would sit inside that mask, lengthening the
    // window that delays the block ISR for no reason. Raw counts are all this
    // function needs to snapshot atomically.
    monitor->last_us10 = 0u;
    monitor->min_us10  = 0u;
    monitor->max_us10  = 0u;

    return valid;
}


/*
 * Convert the raw counts of an already-taken snapshot into 0.1 us units.
 *
 * Split out of _get_load() on purpose: it must run with interrupts RESTORED.
 * Keeping it inside the RX-DMA-IE mask made the mask window long enough to
 * visibly delay the block ISR -- measured as a paired long/short block interval
 * (e.g. 707.6 us followed by 603.0 us, summing to exactly two nominal periods)
 * every time the foreground sampled the load. The delay is real; the ISR catches
 * up on the following block, so no blocks are lost, but it perturbs the very
 * timing being measured.
 */
void nora_spi_i2s_tdm_diag_load_convert( nora_spi_i2s_tdm_load_t* monitor )
{
    if( monitor == NULL )
    {
        return;
    }

    monitor->last_us10 = nora_high_res_timer_count_to_us_x10( monitor->last_count );
    monitor->min_us10  = nora_high_res_timer_count_to_us_x10( monitor->min_count  );
    monitor->max_us10  = nora_high_res_timer_count_to_us_x10( monitor->max_count  );
}


/*
 * Read the two block counters under the caller's ISR mask.
 */
void nora_spi_i2s_tdm_diag_read_counts( const nora_spi_i2s_tdm_diag_t* d,
                                             uint32_t* block_count,
                                             uint32_t* block_deadline_miss_count )
{
    if( d == NULL )
    {
        return;
    }
    if( block_count != NULL )
    {
        *block_count = d->block_count;
    }
    if( block_deadline_miss_count != NULL )
    {
        *block_deadline_miss_count = d->block_deadline_miss_count;
    }
}


//===========================================================
// TDM-active COMBINED-occupancy profiler ("TDMsum") -- engine-wide singleton.
// The hot-path enter/exit/advance/close are static inline in
// nora_spi_i2s_tdm_dspic33ck_diag_fast.h; only the foreground configure/reset/snapshot
// and the shared instance live here. See that header and the public
// nora_spi_i2s_tdm_tdmsum_t for the full concurrency/measurement contract. These do NO
// masking -- the public nora_spi_i2s_tdm_tdmsum_* wrappers in the transport core hold
// every leg's RX-DMA IE around them.
// Compiled only when NORA_TDM_SUMPROF is 1 (see nora_spi_i2s_tdm_conf.h).
//===========================================================
#if NORA_TDM_SUMPROF

nora_spi_i2s_tdm_dspic33ck_sumprof_state_t
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state = {
    0u,     /* window_period_ticks */
    0u,     /* window_end_ticks    */
    0u,     /* busy_start_ticks    */
    0u,     /* busy_ticks          */
    0u,     /* max_busy_ticks      */
    0u,     /* saturated_count     */
    0u,     /* busy_depth          */
    false   /* initialized         */
};

void nora_spi_i2s_tdm_dspic33ck_sumprof_configure( uint32_t now, uint32_t window_period_ticks )
{
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks = window_period_ticks;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks    = now + window_period_ticks;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks    = now;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks          = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks      = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.saturated_count     = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth          = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.initialized         = ( window_period_ticks != 0u );
}

void nora_spi_i2s_tdm_dspic33ck_sumprof_reset( uint32_t now )
{
    // Keep window_period_ticks; re-base the grid and clear depth/accumulators/peaks.
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks = now + g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks = now;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks       = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks   = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.saturated_count  = 0u;
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth       = 0u;
}

void nora_spi_i2s_tdm_dspic33ck_sumprof_snapshot( nora_spi_i2s_tdm_tdmsum_t* out,
                                                   bool clear_peak )
{
    if( out == NULL )
    {
        return;
    }

    out->window_period_ticks = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks;
    out->max_busy_ticks      = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks;
    out->saturated_count     = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.saturated_count;
    out->initialized         = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.initialized;

    if( clear_peak )
    {
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks  = 0u;
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.saturated_count = 0u;
    }
}

#endif // NORA_TDM_SUMPROF
