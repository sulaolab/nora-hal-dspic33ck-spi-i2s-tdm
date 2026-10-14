#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_REG_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_REG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Minimal CK SPI framed-mode register masks for the feasibility spike.
 *
 * CK splits the SPI control registers into 16-bit L/H SFRs. Do not reuse the
 * AK 32-bit SPIxCON1 mask layer directly.
 */

/* ---- SPIxCON1L fields ---- */
#define NORA_SPI_I2S_TDM_CON1L_ENHBUF   (1u << 0)
#define NORA_SPI_I2S_TDM_CON1L_SPIFE    (1u << 1)
#define NORA_SPI_I2S_TDM_CON1L_MCLKEN   (1u << 2)
#define NORA_SPI_I2S_TDM_CON1L_MSTEN    (1u << 5)
#define NORA_SPI_I2S_TDM_CON1L_CKP      (1u << 6)
#define NORA_SPI_I2S_TDM_CON1L_CKE      (1u << 8)
#define NORA_SPI_I2S_TDM_CON1L_MODE16   (1u << 10)
#define NORA_SPI_I2S_TDM_CON1L_MODE32   (1u << 11)
#define NORA_SPI_I2S_TDM_CON1L_SPIEN    (1u << 15)

/* ---- SPIxCON1H fields ---- */
#define NORA_SPI_I2S_TDM_CON1H_FRMCNT_POS   0u
#define NORA_SPI_I2S_TDM_CON1H_FRMCNT_MASK  (0x7u << NORA_SPI_I2S_TDM_CON1H_FRMCNT_POS)
#define NORA_SPI_I2S_TDM_CON1H_FRMSYPW      (1u << 3)
#define NORA_SPI_I2S_TDM_CON1H_FRMPOL       (1u << 5)
#define NORA_SPI_I2S_TDM_CON1H_FRMSYNC      (1u << 6)
#define NORA_SPI_I2S_TDM_CON1H_FRMEN        (1u << 7)
#define NORA_SPI_I2S_TDM_CON1H_IGNTUR       (1u << 12)
#define NORA_SPI_I2S_TDM_CON1H_IGNROV       (1u << 13)
#define NORA_SPI_I2S_TDM_CON1H_AUDEN        (1u << 15)

/* ---- SPIxCON2L fields ---- */
#define NORA_SPI_I2S_TDM_CON2L_WLENGTH_POS   0u
#define NORA_SPI_I2S_TDM_CON2L_WLENGTH_MASK  (0x1Fu << NORA_SPI_I2S_TDM_CON2L_WLENGTH_POS)

/* ---- SPIxSTATL framed-transport health flags ----
 * Same bit positions as the AK SPI module (SPIROV bit6, SPITUR bit8, FRMERR
 * bit12), so the frame-slip / connector-glitch detection ports across unchanged.
 * SPIROV and FRMERR are R/C/HS (software-clearable by writing 0 to the bit);
 * SPITUR is R/HSC (hardware self-clearing, NOT software-writable) and reflects a
 * live transmit-underrun -- observe it, never write it. */
#define NORA_SPI_I2S_TDM_STATL_SPIROV   (1u << 6)    /* receive overflow  (R/C/HS) */
#define NORA_SPI_I2S_TDM_STATL_SRMT     (1u << 7)    /* shift-register empty        */
#define NORA_SPI_I2S_TDM_STATL_SPITUR   (1u << 8)    /* transmit underrun (R/HSC)  */
#define NORA_SPI_I2S_TDM_STATL_FRMERR   (1u << 12)   /* frame-sync error  (R/C/HS) */
/* Bits the frame-slip sampler is allowed to write-0-clear (SPITUR excluded). */
#define NORA_SPI_I2S_TDM_STATL_SW_CLEARABLE \
    (NORA_SPI_I2S_TDM_STATL_SPIROV | NORA_SPI_I2S_TDM_STATL_FRMERR)

/* ---- SPIxSTATH fields ---- */
#define NORA_SPI_I2S_TDM_STATH_TXELM_POS   0u
#define NORA_SPI_I2S_TDM_STATH_TXELM_MASK  (0x3Fu << NORA_SPI_I2S_TDM_STATH_TXELM_POS)
#define NORA_SPI_I2S_TDM_STATH_RXELM_POS   8u
#define NORA_SPI_I2S_TDM_STATH_RXELM_MASK  (0x3Fu << NORA_SPI_I2S_TDM_STATH_RXELM_POS)

/* ---- SPIxIMSKL fields ---- */
#define NORA_SPI_I2S_TDM_IMSKL_SPIRBFEN  (1u << 0)
#define NORA_SPI_I2S_TDM_IMSKL_SPITBEN   (1u << 3)
#define NORA_SPI_I2S_TDM_IMSKL_FRMERREN  (1u << 12)

static inline void nora_spi_i2s_tdm_reg_set16(volatile uint16_t *reg, uint16_t mask)
{
    *reg = (uint16_t)(*reg | mask);
}

static inline void nora_spi_i2s_tdm_reg_clear16(volatile uint16_t *reg, uint16_t mask)
{
    *reg = (uint16_t)(*reg & (uint16_t)~mask);
}

static inline void nora_spi_i2s_tdm_reg_set_or_clear16(volatile uint16_t *reg,
                                                    uint16_t mask,
                                                    bool on)
{
    if (on) {
        nora_spi_i2s_tdm_reg_set16(reg, mask);
    } else {
        nora_spi_i2s_tdm_reg_clear16(reg, mask);
    }
}

static inline void nora_spi_i2s_tdm_reg_write_field16(volatile uint16_t *reg,
                                                   uint16_t mask,
                                                   uint16_t pos,
                                                   uint16_t value)
{
    *reg = (uint16_t)((*reg & (uint16_t)~mask) | ((uint16_t)(value << pos) & mask));
}

/* Bit positions this header hands out are only valid if the DFP still agrees.
 * Included last so the assertions are checked wherever these accessors are used;
 * it emits no code. See the header for why the checks live in their own file. */
#include "nora_spi_i2s_tdm_dspic33ck_reg_assert.h"

#endif /* NORA_SPI_I2S_TDM_DSPIC33CK_REG_H */
