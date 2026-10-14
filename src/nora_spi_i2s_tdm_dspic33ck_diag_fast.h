#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_DIAG_FAST_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_DIAG_FAST_H

//===========================================================
// dsPIC33CK-private ISR fast path for the transport's diagnostics. Deliberately
// SEPARATE from nora_spi_i2s_tdm_diag.h, which carries the neutral name: these hooks
// need the CK DMA status predicates and the CK SPIxSTATL bit masks, and a header whose
// name promises portability must not drag a family's register layout in behind it. A
// portable consumer includes nora_spi_i2s_tdm_diag.h and sees snapshots and out-of-line
// entry points only; the CK backend translation unit opts into this header on top.
//
// The hooks live in a header intentionally. The project builds -Os and the former
// out-of-line diagnostic helpers were still being called roughly 1500 times/s even on
// clean audio; they are short enough to be inlined into the DMA vector, retaining every
// safety diagnostic while removing call setup/return overhead. Callers must pass the real
// per-instance diag object (never NULL).
//
// Naming rule as in nora_dma_dspic33ck_fast.h: <portable name>_hot, supplied by the
// backend header, with the out-of-line twin always present in the neutral header
// (nora_spi_i2s_tdm_diag_note_block / _note_dma_status / _note_errflags). The one
// exception is _should_time_this_block(), which has no portable twin because it is not a
// diagnostic at all -- it is this backend's sampling prescaler, and nothing outside the
// CK RX-block ISR may call it.
//===========================================================

#include <stdint.h>
#include <stdbool.h>
#include "nora_spi_i2s_tdm_diag.h"          // nora_spi_i2s_tdm_diag_t + the out-of-line twins
// The hooks below run on every completed RX block, so they ask the `_hot` predicates
// rather than the out-of-line twins. That is what including this header declares. It is
// also why no DMA status BIT appears here: the status layout is the DMA backend's, and a
// diagnostic that tested bits directly would have to be rewritten for a family whose
// status word is laid out differently.
#include "nora_dma_dspic33ck_fast.h"
#include "nora_spi_i2s_tdm_dspic33ck_reg.h" // SPIROV/SPITUR/FRMERR masks for the hot error hook


static inline __attribute__((always_inline)) bool
nora_spi_i2s_tdm_diag_should_time_this_block( nora_spi_i2s_tdm_diag_t* d )
{
#if (NORA_TDM_ISR_TIMING_SAMPLE_LOG2 == 0)
    // Explicit full-fidelity mode for timing investigations.
    (void)d;
    return true;
#else
    if( d->isr_timing_sample_countdown != 0u )
    {
        d->isr_timing_sample_countdown--;
        return false;
    }

    // Reset to period-1 so reset()/start() (which seeds this field with zero)
    // measures the first audio block immediately, then every 2^N blocks.
    d->isr_timing_sample_countdown =
        (uint8_t)((1u << NORA_TDM_ISR_TIMING_SAMPLE_LOG2) - 1u);
    return true;
#endif
}


static inline __attribute__((always_inline)) void
nora_spi_i2s_tdm_diag_note_dma_status_hot( nora_spi_i2s_tdm_diag_t* d,
                                                 nora_dma_status_t dma_stat )
{
    // This runs before the completed-half check.  In particular, an overrun-only
    // DMA IRQ has no callback buffer, but must still remain visible to status.
    //
    // The stored copy narrows on purpose: the snapshot came from a 16-bit register, so
    // the internal last-status field stays 16-bit and this stays one volatile 16-bit
    // store per RX block. The public status API widens it again.
    d->rx_dma_last_status = (uint16_t)dma_stat;
    if( nora_dma_status_has_overrun_hot( dma_stat ) )
    {
        d->rx_dma_overrun_count++;
    }
    if( !nora_dma_status_has_completed_half_hot( dma_stat ) )
    {
        d->rx_dma_other_irq_count++;
    }
}


static inline __attribute__((always_inline)) void
nora_spi_i2s_tdm_diag_note_block_hot( nora_spi_i2s_tdm_diag_t* d )
{
    // Retain an exact 32-bit block count; only the timing profiler is sampled.
    d->block_count++;
}


static inline __attribute__((always_inline)) void
nora_spi_i2s_tdm_diag_note_errflags_hot( nora_spi_i2s_tdm_diag_t* d,
                                               uint16_t flags )
{
    // SPI flags are sampled/acknowledged every block by the caller.  The counts
    // therefore retain their former exact meaning: number of affected blocks.
    if( ( flags & NORA_SPI_I2S_TDM_STATL_SPIROV ) != 0u ) { d->err_rov_block_count++; }
    if( ( flags & NORA_SPI_I2S_TDM_STATL_SPITUR ) != 0u ) { d->err_tur_block_count++; }
    if( ( flags & NORA_SPI_I2S_TDM_STATL_FRMERR ) != 0u )
    {
        d->err_frm_block_count++;
        d->frmerr_consecutive_blocks++;
    }
    else if( d->frmerr_consecutive_blocks != 0u )
    {
        // A clean block still breaks a FRMERR run exactly as before.  Avoid the
        // old unconditional 32-bit store of zero on the overwhelmingly common
        // clean/zero case.
        d->frmerr_consecutive_blocks = 0u;
    }
}


#if NORA_TDM_SUMPROF
//===========================================================
// Engine-wide TDMsum profiler: mutable state + the ISR fast path.
//
// This state is deliberately NOT in nora_spi_i2s_tdm_diag.h: portable consumers see the
// nora_spi_i2s_tdm_tdmsum_t snapshot only, never this mutable implementation state. It is
// engine-wide (one instance for all legs) because the quantity being measured is the UNION
// over legs -- a per-leg copy could not express "TDM1 and TDM2 overlapping count once".
//
// The time base is whatever nora_high_res_timer_get_count() returns, which on this family is
// SCCP1 (dsPIC33CK has no Timer2/3); the caller passes `now` in so the ISR pays for exactly
// one timer read per hook. All fields are raw counts of that timer -- no unit conversion
// happens in the ISR.
//===========================================================

// Bounds the per-call window-advance loop. Steady state advances 0-1 windows per ISR edge;
// after a long stopped gap the grid is re-based rather than walking every empty window.
#define NORA_SPI_I2S_TDM_DSPIC33CK_SUMPROF_MAX_CATCHUP  (4u)

typedef struct {
    volatile uint32_t window_period_ticks;
    volatile uint32_t window_end_ticks;
    volatile uint32_t busy_start_ticks;
    volatile uint32_t busy_ticks;
    volatile uint32_t max_busy_ticks;
    volatile uint32_t saturated_count;
    volatile uint8_t  busy_depth;
    volatile bool     initialized;
} nora_spi_i2s_tdm_dspic33ck_sumprof_state_t;

// Defined by nora_spi_i2s_tdm_dspic33ck_diag.c. External linkage is needed only because the
// inline ISR hooks and the foreground snapshot code live in separate backend translation units.
extern nora_spi_i2s_tdm_dspic33ck_sumprof_state_t
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state;

// Signed difference, so the comparison stays correct across the counter's 32-bit wrap
// (~42.9 s at Fcy 100 MHz) instead of failing once per wrap.
static inline bool nora_spi_i2s_tdm_dspic33ck_sumprof_reached( uint32_t now,
                                                                uint32_t end )
{
    return (int32_t)( now - end ) >= 0;
}

static inline void nora_spi_i2s_tdm_dspic33ck_sumprof_close_window( void )
{
    uint32_t busy = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks;

    if( busy > g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks )
    {
        busy = g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks;
    }
    if( busy > g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks )
    {
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.max_busy_ticks = busy;
    }
    if( busy >= g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks )
    {
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.saturated_count++;
    }
}

static inline void nora_spi_i2s_tdm_dspic33ck_sumprof_advance( uint32_t now )
{
    uint8_t guard = 0u;

    if( !g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.initialized ||
        ( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks == 0u ) )
    {
        return;
    }

    while( nora_spi_i2s_tdm_dspic33ck_sumprof_reached(
               now, g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks ) )
    {
        if( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth != 0u )
        {
            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks +=
                (uint32_t)( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks -
                            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks );
            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks =
                g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks;
        }

        nora_spi_i2s_tdm_dspic33ck_sumprof_close_window();

        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks = 0u;
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks +=
            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks;

        if( ++guard >= NORA_SPI_I2S_TDM_DSPIC33CK_SUMPROF_MAX_CATCHUP )
        {
            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_end_ticks =
                now + g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.window_period_ticks;
            g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks = 0u;
            if( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth != 0u )
            {
                g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks = now;
            }
            break;
        }
    }
}

// busy_depth, not a flag: the legs' RX ISRs do not preempt each other on this core, but a
// second leg's ISR can still open while the first is pending, and the union must close only
// when the LAST one leaves.
static inline void nora_spi_i2s_tdm_dspic33ck_sumprof_enter( uint32_t now )
{
    nora_spi_i2s_tdm_dspic33ck_sumprof_advance( now );

    if( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth == 0u )
    {
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks = now;
    }
    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth++;
}

static inline void nora_spi_i2s_tdm_dspic33ck_sumprof_exit( uint32_t now )
{
    nora_spi_i2s_tdm_dspic33ck_sumprof_advance( now );

    if( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth == 0u )
    {
        return;
    }

    g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth--;

    if( g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_depth == 0u )
    {
        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_ticks +=
            (uint32_t)( now -
                        g_nora_spi_i2s_tdm_dspic33ck_sumprof_state.busy_start_ticks );
    }
}

// Foreground half, out of line in nora_spi_i2s_tdm_dspic33ck_diag.c. These do NO masking:
// the public nora_spi_i2s_tdm_tdmsum_* wrappers in the transport core hold every leg's
// RX-DMA IE around them.
void nora_spi_i2s_tdm_dspic33ck_sumprof_configure( uint32_t now,
                                                    uint32_t window_period_ticks );
void nora_spi_i2s_tdm_dspic33ck_sumprof_reset( uint32_t now );
void nora_spi_i2s_tdm_dspic33ck_sumprof_snapshot( nora_spi_i2s_tdm_tdmsum_t* out,
                                                   bool clear_peak );

#endif // NORA_TDM_SUMPROF


#endif // NORA_SPI_I2S_TDM_DSPIC33CK_DIAG_FAST_H
