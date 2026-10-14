#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_FS_CLC_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_FS_CLC_H

//===========================================================
// CLC1 50%-duty frame-sync generator (silicon helper of the SPI/I2S/TDM HAL).
//
// This is the CK sibling of AK's CLC10/RPV8 module, remapped to the CK64MC105/CK256MP508
// silicon: CLC1 instead of CLC10, virtual pin RPV0 (RP176) instead of RPV8, and a
// device-table pin lookup instead of AK's numeric RP range (see nora_spi_i2s_tdm_dspic33ck_fs_clc.c
// for why -- CK64MC105's remappable pins are not contiguous above RP61).
//
// For a TDM MASTER configured with FS_50PCT, the SPI emits a 1-BCLK half-frame marker
// (FRMSYPW=0, FRMCNT=slots/2) on its FRMSYNC (SSx) output. This module turns that marker
// into a ~50%-duty FS by toggling CLC1 (a J-K flip-flop, J=K=1) on every marker edge, and
// puts the result on the SAME external FS pin the board already routed -- so the
// application never deals with CLC/PPS.
//
// It routes PPS through hal_gpio/nora_pps.h (2026-08-03). It used to be self-contained
// (xc.h only) with its own IOLOCK sequence and its own per-device RPORx-slot -> RPn tables,
// on the grounds that the transport HAL should stay portable. That cost more than
// it bought: the private copy of the register map is where the non-contiguous-RP bug above
// lived, and a second copy of a device fact is a second chance to get it wrong. The PPS HAL
// is a sibling within the same hal_gpio family and travels with it, so the dependency is
// inside the family rather than across it.
//
// What it does on engage():
//   1. Resolve this instance's FRMSYNC signal (hw_get_ss_pps_output -> a PPS output enum).
//   2. Ask the PPS HAL which physical pin carries it (nora_pps_find_output_rp) -- that
//      pin IS the external FS. (On a restart it instead finds the pin already routed to
//      CLC1OUT and reuses it.)
//   3. Route FRMSYNC (SSx) internally to virtual pin RPV0 -- no jumper, no extra pad.
//   4. Configure CLC1 as a J-K flip-flop clocked by RPV0/CLCINA (gate roles: CLK=Gate1,
//      J=Gate2, K=Gate4, R=Gate3), with a known initial state.
//   5. Repoint that external FS pin from FRMSYNC to CLC1OUT.
//
// RESOURCE OWNERSHIP: there is ONE CLC1 used this way. It is owned by the instance/clock-
// domain that engages it. A different instance trying to engage while it is owned gets
// _BUSY (the core maps that to ERR_CLC). Sharing one CLC1 FS across several co-clocked,
// co-format, phase-aligned instances (fan CLC1OUT to several FS pins) is a future extension
// tied to the multi-instance clock-domain work; today a second independent domain is
// rejected.
//
// DEVICE: implemented for dsPIC33CK64MC105 and dsPIC33CK256MP508 (both have CLC1 + RPV0);
// nora_spi_i2s_tdm_dspic33ck_fs_clc.c #errors at build time for any other NORA_SPI_I2S_TDM_DSPIC33CK_DEVICE
// rather than silently using a wrong pin table.
//===========================================================

#include "nora_spi_i2s_tdm_dspic33ck_hw.h"   // tdm_spi_inst_t

typedef enum {
    NORA_SPI_I2S_TDM_FS_CLC_OK = 0,
    NORA_SPI_I2S_TDM_FS_CLC_BUSY,       // CLC1 already owned by a different instance/domain
    NORA_SPI_I2S_TDM_FS_CLC_NO_FS_PIN,  // FS/FRMSYNC not on any physical pin (or no CLC1 on device)
} nora_spi_i2s_tdm_fs_clc_result_t;

// Engage the CLC1 50%-FS generator for `owner` (a TDM master). Idempotent for the current
// owner (also handles re-start after release). Returns _BUSY if another owner holds CLC1,
// _NO_FS_PIN if the FS pin can't be resolved / CLC1 absent. Call AFTER the board has
// routed FRMSYNC->FS pin (open()) and BEFORE enabling the SPI module.
nora_spi_i2s_tdm_fs_clc_result_t
    nora_spi_i2s_tdm_fs_clc_engage( tdm_spi_inst_t owner );

// Release CLC1 if `owner` holds it: disables the flip-flop AND restores the external FS pin
// from CLC1OUT back to its original FRMSYNC (SSx) output, so a runtime reconfigure
// (FS_50PCT -> stop -> FS_PULSE -> start) leaves the SPI driving the FS pin directly again.
// No-op if `owner` is not the current holder.
void nora_spi_i2s_tdm_fs_clc_release( tdm_spi_inst_t owner );

#endif // NORA_SPI_I2S_TDM_DSPIC33CK_FS_CLC_H
