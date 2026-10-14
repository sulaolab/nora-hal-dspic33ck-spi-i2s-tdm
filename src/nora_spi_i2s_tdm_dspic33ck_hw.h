#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_HW_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_HW_H

//===========================================================
// nora_spi_i2s_tdm_dspic33ck_hw.{c,h} = the SILICON layer of the SPI/I2S/TDM HAL,
// kept separate from the transport core. CK sibling of
// nora_spi_i2s_tdm_dspic33ak_hw.{c,h}:
// same instance-decoupled contract, rewritten for the CK SPI peripheral (16-bit
// L/H control registers SPIxCON1L/CON1H/CON2L, SPIxBUFL, SPIxBRGL, SPIxIMSKL) and
// the CK DMA HAL. hw.c owns the device-facts table (s_spi_dev[]) and every
// function that programs one physical SPI in framed (I2S/TDM) mode + arms its DMA.
//
// The functions take a tdm_spi_inst_t (which physical SPI) plus the raw DMA
// channels/buffers, NOT the transport's instance/leg struct -- a clean "drive the
// silicon" boundary identical to the AK layer.
//===========================================================

#include <stdint.h>
#include <stdbool.h>
#include "nora_spi_i2s_tdm.h"   // nora_spi_i2s_tdm_config_t / _clock_role_t
#include "nora_dma.h"           // nora_dma_channel_t (the DMA channels this layer arms)
#include "nora_pps.h"           // nora_pps_output_t (frame-sync signal identity)
#include "nora_spi_i2s_tdm_dspic33ck_reg.h" // SPIxSTATL health-bit masks used by the hot sampler

//===========================================================
// Device identity (HAL-owned adapter). Maps the toolchain's -mcpu predefined macro to an
// opaque, module-prefixed tag in ONE place; this silicon layer selects the SPI device-facts
// table / instance count on it. Self-contained: depends only on the compiler macro, never on
// app config. Add a sibling part by OR-ing it into one arm here -- the single part-number
// change point. Tag values arbitrary; compare with == only (never order / arithmetic).
// This HAL hard-#errors on an unsupported device.
//
// It lives HERE, not in the neutral nora_spi_i2s_tdm.h contract, because the names carry a
// _DSPIC33CK_ tag: chip-specific identifiers belong to the backend, and the dsPIC33AK HAL
// likewise keeps device selection out of its neutral header. Every backend translation unit
// includes this file, so the unsupported-device #error still fires where it matters, while a
// consumer that includes only the contract header no longer sees a chip-tagged macro.
//===========================================================
#define NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK256MP508   (1)
#define NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK64MC105    (2)
#define NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK256MC005   (3)
#define NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK1024MP710  (4)

#if   defined(__dsPIC33CK1024MP710__)
  #define NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK1024MP710
#elif defined(__dsPIC33CK256MP508__)
  #define NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE    NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK256MP508
#elif defined(__dsPIC33CK64MC105__)
  #define NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE    NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK64MC105
#elif defined(__dsPIC33CK256MC005__)
  #define NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE    NORA_SPI_I2S_TDM_DSPIC33CK_DEV_CK256MC005
#else
  #error "Unsupported device -- the SPI/I2S/TDM HAL expects __dsPIC33CK256MP508__, __dsPIC33CK64MC105__, __dsPIC33CK256MC005__, or __dsPIC33CK1024MP710__."
#endif

// Physical SPI instances present on the CK256MP508 (data sheet): SPI1/2/3.
// TDM_SPI_INST_COUNT bounds an instance in the core.
typedef enum {
    TDM_SPI1 = 0,
    TDM_SPI2 = 1,
    TDM_SPI3 = 2,
    TDM_SPI_INST_COUNT
} tdm_spi_inst_t;

// ---- Per-physical-SPI silicon operations (no instance-struct knowledge) ----
// apply_config writes SPIxCON1L/CON1H/CON2L/BRGL from the validated config and leaves
// DMA-trigger events + the module OFF. dma_config configures + enables that SPI's RX
// and TX DMA channels (RX: SPIxBUFL->rx_buf, TX: tx_buf->SPIxBUFL) and returns false
// if the DMA controller rejects a channel. dma_trigger_enable toggles the SPIxIMSKL
// event->DMA enables; module_enable toggles SPIxCON1L.SPIEN; irq_clear_flags clears the
// CPU RX/TX flags; soft_stop disables triggers + masks CPU IRQs + clears SPIEN.
void nora_spi_i2s_tdm_hw_apply_config( tdm_spi_inst_t inst,
                                            const nora_spi_i2s_tdm_config_t* cfg );
bool nora_spi_i2s_tdm_hw_dma_config( tdm_spi_inst_t inst,
                                          nora_dma_channel_t rx_dma_ch,
                                          nora_dma_channel_t tx_dma_ch,
                                          nora_tdm_slot_t* rx_buffer,
                                          nora_tdm_slot_t* tx_buffer,
                                          uint32_t buffer_slot_count );
void nora_spi_i2s_tdm_hw_dma_trigger_enable( tdm_spi_inst_t inst, bool enable );
void nora_spi_i2s_tdm_hw_module_enable( tdm_spi_inst_t inst, bool enable );
void nora_spi_i2s_tdm_hw_irq_clear_flags( tdm_spi_inst_t inst );
void nora_spi_i2s_tdm_hw_soft_stop( tdm_spi_inst_t inst );

/*
 * Resolve an instance's SPIxSTATL address while the transport is stopped or
 * being started.  This is intentionally a non-hot silicon-table lookup: the
 * core caches the result in its leg descriptor before it enables DMA/SPI.
 * NULL denotes an invalid physical-SPI enum.
 *
 * The pointer is an internal HAL implementation detail, not an invitation for
 * application code to write SPIxSTATL.  Its only consumer is the inline sampler
 * below, which preserves the silicon-specific W0C acknowledgement protocol.
 */
volatile uint16_t *nora_spi_i2s_tdm_hw_get_statl( tdm_spi_inst_t inst );

/*
 * Sample + ack a previously validated SPIxSTATL register once per completed
 * RX block.  The caller must pass the non-NULL pointer returned by
 * nora_spi_i2s_tdm_hw_get_statl(); that startup-time invariant removes a
 * table lookup, enum validation, function call, and return from every DMA ISR.
 *
 * This is NOT a lower-quality diagnostic mode.  It reads every block and
 * returns exactly the same SPIROV/SPITUR/FRMERR mask as the former generic
 * function.  SPIROV and FRMERR remain acknowledged on every block, while
 * SPITUR remains strictly observed-only because the peripheral clears it.
 *
 * The write must never replay the full `status` snapshot.  A sticky flag can
 * assert between the volatile read and write; replaying stale ones would clear
 * that newly asserted error.  The expression below writes only the
 * software-clearable bit mask, with only the flags observed in THIS read set
 * to zero (the required W0C-safe acknowledgement).
 */
static inline __attribute__((always_inline)) uint16_t
nora_spi_i2s_tdm_hw_sample_ack_errflags_hot( volatile uint16_t *statl )
{
    const uint16_t status = *statl;
    const uint16_t observed = status
                        & ( NORA_SPI_I2S_TDM_STATL_SPIROV
                          | NORA_SPI_I2S_TDM_STATL_SPITUR
                          | NORA_SPI_I2S_TDM_STATL_FRMERR );
    const uint16_t clearable = observed & NORA_SPI_I2S_TDM_STATL_SW_CLEARABLE;

    if( clearable != 0u )
    {
        *statl = (uint16_t)(NORA_SPI_I2S_TDM_STATL_SW_CLEARABLE & (uint16_t)~clearable);
    }
    return observed;
}

// The PPS HAL's output enum for one SPI instance's frame-sync (SSx = FRMSYNC output in
// framed master mode). Used by the CLC 50%-FS module (master, Stage 2), which hands it
// straight to nora_pps_find_output_rp() to locate the FS pin and to
// nora_pps_route_output() to move the signal. Returns false only if inst is out of
// range -- whether the DEVICE has that SSx is the PPS HAL's question, answered when the
// enum is used (it carries the _RPOUT_SSx / _RPOUT_SSxOUT ladder).
bool nora_spi_i2s_tdm_hw_get_ss_pps_output( tdm_spi_inst_t inst,
                                                 nora_pps_output_t* output );

#endif // NORA_SPI_I2S_TDM_DSPIC33CK_HW_H
