//===========================================================
// CK CLC 50%-duty frame-sync generator -- see nora_spi_i2s_tdm_dspic33ck_fs_clc.h.
// CK sibling of the AK CLC10 module, remapped to the CK256MP508 silicon.
//
// Purpose (TDM MASTER + FS_50PCT): the SPI emits a 1-BCLK half-frame marker on its
// FRMSYNC (SSx) output (FRMSYPW=0, FRMCNT=slots/2). A CLC configured as a J-K flip-flop
// (J=K=1) toggles on every marker edge, producing a ~50%-duty FS, which is put back on the
// SAME external FS pin the board routed -- so the application never deals with CLC/PPS.
//
// CK path (differs from AK's single "CLC reads Virtual Pin 8" data source):
//   1. Resolve this instance's FRMSYNC signal (hw_get_ss_pps_output -> a PPS output enum).
//   2. Ask the PPS HAL which physical pin currently carries it (nora_pps_find_output_rp)
//      -- that pin IS the external FS output, because the board routed FRMSYNC there.
//   3. Route FRMSYNC (SSx) to virtual pin RPV0 -- internal, no pad.
//   4. Route the CLC's input A from RPV0, and select DS1 = CLCINA.
//   5. Configure CLC1 as a J-K flip-flop clocked by CLCINA, then repoint the external FS
//      pin from FRMSYNC to CLC1OUT.
//
// PPS goes THROUGH hal_gpio/nora_pps.h (2026-08-03). This file used to do its own
// PPS: its own IOLOCK sequence, its own per-device RPORx-slot -> RPn tables, and its own
// bank arithmetic to reach _RPnnR. All three are gone -- the HAL owns the register map
// (one flat #ifdef'd switch per RP, so a pin the device lacks cannot be addressed at all)
// and the lock gate. What is left here is the CLC configuration, which is genuinely this
// module's own. The three PPS operations this needs -- route to a virtual pin, read an
// input from a virtual pin, and reverse-look-up "which pad carries this signal" -- were
// added to the PPS HAL for this, rather than kept private here.
//
// Register-layout facts below (MODE=0b110, DS1=0b000, and RPV0 as a CLCINAR source) are
// checked against DS70005399D (CK64MC105) / DS70005349 (CK256MP508) Register 21-1
// (CLCxCONL.MODE), Register 21-3 (CLCxSELL.DS1), and Table 8-4 (input PPS pin list incl.
// virtual pins).
//===========================================================

#include "nora_spi_i2s_tdm_dspic33ck_fs_clc.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nora_pps.h"

#define FS_CLC_MODE_JK_FF_WITH_R   (0b110u)   // CLC1CONL.MODE : J-K flip-flop with reset
#define FS_CLC_DS1_CLCINA          (0u)       // CLC1SELL.DS1 : CLCINA
#define FS_CLC_MARKER_RP           NORA_PPS_RPV0   // FRMSYNC marker -> CLC input, padless


//===========================================================
// Variables (ownership of the single CLC1 used for FS)
//===========================================================
static bool                   s_claimed;
static tdm_spi_inst_t         s_owner;
static nora_gpio_rp_t    s_fs_rp;      // physical FS pin repointed to CLC1OUT (restore on release)
static nora_pps_output_t s_ss_output;  // the FRMSYNC (SSx) signal that pin carried before


//===========================================================
// Function Prototype (private)
//===========================================================
static void fs_clc_configure_clc(void);


//===========================================================
// Global Function
//===========================================================
nora_spi_i2s_tdm_fs_clc_result_t
    nora_spi_i2s_tdm_fs_clc_engage( tdm_spi_inst_t owner )
{
    nora_pps_output_t ss_output;
    nora_gpio_rp_t    fs_rp;
    bool                   ok;

    if (s_claimed && (s_owner != owner))
    {
        return NORA_SPI_I2S_TDM_FS_CLC_BUSY;
    }
    if (!nora_spi_i2s_tdm_hw_get_ss_pps_output(owner, &ss_output))
    {
        return NORA_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;   // instance has no FRMSYNC output
    }

    if (nora_pps_find_output_rp(ss_output, &fs_rp))
    {
        // First engage: board routed FRMSYNC -> this physical pin. Send FRMSYNC to RPV0
        // (CLC input) and repoint the external pin from FRMSYNC to CLC1OUT. Both routes are
        // to targets that are known good (RPV0 is a fixed virtual pin; fs_rp just came back
        // from a successful lookup) -- a false return means the PPS HAL and this call
        // disagree about what exists, not a runtime condition, so it reports NO_FS_PIN
        // rather than being silently accepted.
        ok  = nora_pps_route_output(ss_output, FS_CLC_MARKER_RP);          // marker -> RPV0
        ok &= nora_pps_route_input(NORA_PPS_INPUT_CLCINA,
                                        FS_CLC_MARKER_RP);                     // CLC in A <- RPV0
        ok &= nora_pps_route_output(NORA_PPS_OUTPUT_CLC1, fs_rp);     // FS pin <- CLC1OUT
    }
    else if (nora_pps_find_output_rp(NORA_PPS_OUTPUT_CLC1, &fs_rp))
    {
        // Re-engage (idempotent for the current owner): FRMSYNC is no longer on a physical
        // pin because a previous engage repointed it to CLC1OUT. That CLC1OUT pin is the FS
        // pin, so keep it, and just re-assert the marker path.
        ok  = nora_pps_route_output(ss_output, FS_CLC_MARKER_RP);
        ok &= nora_pps_route_input(NORA_PPS_INPUT_CLCINA, FS_CLC_MARKER_RP);
    }
    else
    {
        return NORA_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;   // FRMSYNC is on no pin at all
    }

    if (!ok)
    {
        return NORA_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;
    }

    fs_clc_configure_clc();
    s_fs_rp     = fs_rp;
    s_ss_output = ss_output;
    s_owner     = owner;
    s_claimed   = true;
    return NORA_SPI_I2S_TDM_FS_CLC_OK;
}

void nora_spi_i2s_tdm_fs_clc_release( tdm_spi_inst_t owner )
{
    if (!s_claimed || (s_owner != owner))
    {
        return;
    }
    CLC1CONLbits.LCEN = 0u;    // disable the flip-flop

    // Restore the external FS pin from CLC1OUT back to its original FRMSYNC (SSx) output.
    // s_fs_rp came from a successful lookup at engage() and cannot have changed since, so
    // the return is not re-checked here (there is also no useful recovery from release()).
    (void)nora_pps_route_output(s_ss_output, s_fs_rp);

    s_claimed = false;
}


//===========================================================
// Local Function
//===========================================================

// CLC1 as a J-K flip-flop toggled by the CLCINA (RPV0) marker.
// Gate roles: Gate1=CLK (Data1 true), Gate2=J, Gate4=K, Gate3=R(async reset). An empty
// gate outputs 0; setting GxPOL inverts it to a constant 1, so J/K are forced to 1 without
// consuming an input, and R is pulsed to 1 at enable then released.
static void fs_clc_configure_clc(void)
{
    CLC1CONL = 0u;                                 // LCEN=0 while configuring; all bits 0
    CLC1CONH = 0u;
    CLC1SELL = 0u;
    CLC1SELLbits.DS1   = FS_CLC_DS1_CLCINA;        // Data1 = CLCINA (the FRMSYNC marker via RPV0)
    CLC1GLSL           = 0u;
    CLC1GLSH           = 0u;
    CLC1GLSLbits.G1D1T = 1u;                       // Gate1 = Data1 (true) = CLK

    CLC1CONLbits.MODE  = FS_CLC_MODE_JK_FF_WITH_R; // 0b110 : J-K flip-flop with R
    CLC1CONHbits.G1POL = 0u;                       // CLK non-inverted (toggle on marker edge)
    CLC1CONHbits.G2POL = 1u;                       // J = 1 (empty gate 0 -> inverted to 1)
    CLC1CONHbits.G4POL = 1u;                       // K = 1
    CLC1CONHbits.G3POL = 1u;                       // R = 1 (assert reset for a known initial Q)
    CLC1CONLbits.LCPOL = 0u;                       // FS starts Low; first marker drives it High
    CLC1CONLbits.LCOE  = 1u;                       // drive CLC1OUT onto the (PPS-routed) FS pin
    CLC1CONLbits.LCEN  = 1u;                       // enable: R still asserted -> FF held reset
    CLC1CONHbits.G3POL = 0u;                        // release R: J=K=1 -> FF toggles each edge
}
