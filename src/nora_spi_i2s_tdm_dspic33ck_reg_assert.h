#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_REG_ASSERT_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_REG_ASSERT_H

/*
 * nora_spi_i2s_tdm_dspic33ck_reg_assert.h
 * ----------------------------------
 *
 * Static assertions that the installed DFP still places the SFR bits where
 * the SPI/I2S/TDM HAL assumes they are. Nothing here generates code -- if every
 * assertion holds, this header compiles to nothing.
 *
 * WHY IT EXISTS, AND WHY IT IS A SEPARATE FILE
 * --------------------------------------------
 * These checks came from src/spikes/ck_spi_tdm_feasibility_probe.c, a compile-only feasibility probe
 * written before the HAL existed to prove the DFP exposed what a CK port would
 * need. The probe was in NO build -- build.ps1 never referenced src/spikes -- so
 * all 27 of its assertions had never once been compiled. A safety net that
 * looks present and is in fact inert is worse than none, because it invites the
 * assumption that a DFP change would be caught. Now they are in a header that the
 * HAL includes, so they run on every build and a moved bit position stops the
 * build instead of silently mis-driving the peripheral.
 *
 * Kept out of the HAL sources rather than pasted into them because the assertion
 * SET is device-scoped while the HAL is meant to be device-agnostic: a new CK
 * variant can supply its own without editing HAL logic, and this file can be
 * dropped or replaced when the HAL is reused on another target. (xc.h already makes the
 * HAL DFP-dependent, so keeping DFP knowledge here is not a new dependency --
 * only a tidier place for it.)
 *
 * The probe's other half -- functions that poked the registers directly and
 * hard-coded DM330030 pin numbers -- was deleted, not moved: it has no callers,
 * it is superseded by the HAL it was written to justify, and it did not even
 * compile for CK64MC105 (TRISDbits.TRISD2 does not exist in that package).
 */

#include <xc.h>

#if _SPI1CON1L_ENHBUF_POSITION != 0
#error "Unexpected SPI1CON1L.ENHBUF position"
#endif
#if _SPI1CON1L_SPIFE_POSITION != 1
#error "Unexpected SPI1CON1L.SPIFE position"
#endif
#if _SPI1CON1L_MCLKEN_POSITION != 2
#error "Unexpected SPI1CON1L.MCLKEN position"
#endif
#if _SPI1CON1L_MSTEN_POSITION != 5
#error "Unexpected SPI1CON1L.MSTEN position"
#endif
#if _SPI1CON1L_CKP_POSITION != 6
#error "Unexpected SPI1CON1L.CKP position"
#endif
#if _SPI1CON1L_CKE_POSITION != 8
#error "Unexpected SPI1CON1L.CKE position"
#endif
#if _SPI1CON1L_MODE16_POSITION != 10
#error "Unexpected SPI1CON1L.MODE16 position"
#endif
#if _SPI1CON1L_MODE32_POSITION != 11
#error "Unexpected SPI1CON1L.MODE32 position"
#endif
#if _SPI1CON1L_SPIEN_POSITION != 15
#error "Unexpected SPI1CON1L.SPIEN position"
#endif
#if _SPI1CON1H_FRMCNT_POSITION != 0
#error "Unexpected SPI1CON1H.FRMCNT position"
#endif
#if _SPI1CON1H_FRMSYPW_POSITION != 3
#error "Unexpected SPI1CON1H.FRMSYPW position"
#endif
#if _SPI1CON1H_FRMPOL_POSITION != 5
#error "Unexpected SPI1CON1H.FRMPOL position"
#endif
#if _SPI1CON1H_FRMSYNC_POSITION != 6
#error "Unexpected SPI1CON1H.FRMSYNC position"
#endif
#if _SPI1CON1H_FRMEN_POSITION != 7
#error "Unexpected SPI1CON1H.FRMEN position"
#endif
#if _SPI1CON1H_IGNTUR_POSITION != 12
#error "Unexpected SPI1CON1H.IGNTUR position"
#endif
#if _SPI1CON1H_IGNROV_POSITION != 13
#error "Unexpected SPI1CON1H.IGNROV position"
#endif
#if _SPI1CON1H_AUDEN_POSITION != 15
#error "Unexpected SPI1CON1H.AUDEN position"
#endif
#if _SPI1CON2L_WLENGTH_POSITION != 0
#error "Unexpected SPI1CON2L.WLENGTH position"
#endif
#if _SPI1STATH_TXELM_POSITION != 0
#error "Unexpected SPI1STATH.TXELM position"
#endif
#if _SPI1STATH_RXELM_POSITION != 8
#error "Unexpected SPI1STATH.RXELM position"
#endif
#if _SPI1IMSKL_SPIRBFEN_POSITION != 0
#error "Unexpected SPI1IMSKL.SPIRBFEN position"
#endif
#if _SPI1IMSKL_SPITBEN_POSITION != 3
#error "Unexpected SPI1IMSKL.SPITBEN position"
#endif
#if _SPI1IMSKL_FRMERREN_POSITION != 12
#error "Unexpected SPI1IMSKL.FRMERREN position"
#endif
#if _RPINR20_SDI1R_POSITION != 0
#error "Unexpected RPINR20.SDI1R position"
#endif
#if _RPINR20_SCK1R_POSITION != 8
#error "Unexpected RPINR20.SCK1R position"
#endif
#if _RPINR21_SS1R_POSITION != 0
#error "Unexpected RPINR21.SS1R position"
#endif
#if _RPOUT_SDO1 != 5
#error "Unexpected PPS output code for SDO1"
#endif

#endif /* NORA_SPI_I2S_TDM_DSPIC33CK_REG_ASSERT_H */
