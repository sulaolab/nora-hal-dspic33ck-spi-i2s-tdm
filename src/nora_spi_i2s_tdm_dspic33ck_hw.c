//===========================================================
// INCLUDES
//===========================================================
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nora_dma.h"               // CK DMA channel config/enable
#include "nora_spi_i2s_tdm_dspic33ck_reg.h"   // CK SPI framed-mode register masks (L/H split)
#include "nora_spi_i2s_tdm_dspic33ck_hw.h"


//===========================================================
// Definition
//===========================================================

// ---- Debug master switch (silicon layer) ----
//#define ENA_TDM_DBG
#if defined(ENA_TDM_DBG)
  #include <stdio.h>
  #define TDM_DBG_PRINTF(...)   printf(__VA_ARGS__)
#else
  #define TDM_DBG_PRINTF(...)   ((void)0)
#endif

#define PRIO_TDM_DMA              (4)

/*
 * The DMA trigger a leg needs is now named as a peripheral leg (nora_dma_trigger_t),
 * not transcribed as a register code. This file used to hold the CHSEL numbers from
 * "TABLE 10-1: DMA CHANNEL TRIGGER SOURCES"; they moved to dma_trigger_to_chsel() in
 * src/hal_dma/nora_dma_dspic33ck.c, which is the single place that encoding lives and
 * also the place that refuses SPI3 on a device that has none.
 *
 * That comment is worth following before assuming this was cosmetic: CHSEL is NOT the
 * CPU interrupt vector number, and conflating the two was this transport's actual
 * first-silicon failure on both devices -- a transport that configured perfectly,
 * reported success, and moved zero elements. Naming the leg instead of the number is
 * what makes that failure unreachable from here.
 */


//===========================================================
// Enum & Struct typedef (silicon facts)
//===========================================================
/*
 * There is deliberately no CPU-IRQ descriptor here.
 *
 * A `{ &IECn, mask, &IFSn, mask }` row per instance is the shape this HAL used to carry,
 * and it cannot be written one bit at a time: with the register in a pointer and the bit
 * in a runtime mask the compiler has to build the whole word -- load, or/and, store. IFSx
 * and IECx are shared by every peripheral on the part, so that store puts back whatever
 * the word held at the load. On EV88G73A IEC0 holds the 1 ms tick, both audio DMA legs,
 * the load-monitor time base, SPI1 RX/TX and the console UART all together, so there is
 * no such thing as touching "the SPI's interrupt bit" there.
 *
 * The CPU flags are therefore written through the DFP bit aliases (`_SPI1RXIE`) with a
 * literal value, in hw_spi_irq_bits_enable() / hw_spi_irq_bits_clear_flags() below, which
 * compiles to a single bset.b/bclr.b. That also removes the per-part bank knowledge the
 * table used to carry (SPI1 in IFS0, SPI2 in IFS1, SPI3 in IFS3): the alias names the bit
 * wherever the DFP puts it.
 */

// CK SPI control registers are split into 16-bit L/H halves.
typedef struct {
    volatile void     *spi_buf;     // &SPIxBUFL (16-bit DMA data port)
    volatile uint16_t *con1l;       // &SPIxCON1L (MSTEN/MODE32/SPIFE/CKP/CKE/MCLKEN/ENHBUF/SPIEN)
    volatile uint16_t *con1h;       // &SPIxCON1H (FRMEN/FRMCNT/FRMSYPW/FRMPOL/FRMSYNC/AUDEN/IGNROV/IGNTUR)
    volatile uint16_t *con2l;       // &SPIxCON2L (WLENGTH)
    volatile uint16_t *brgl;        // &SPIxBRGL
    volatile uint16_t *imskl;       // &SPIxIMSKL (event->DMA enables)
    volatile uint16_t *statl;       // &SPIxSTATL (SPIROV/SPITUR/FRMERR framed-transport health)
    // Which peripheral leg fires the DMA channel. A logical trigger, not a register
    // code -- the DMA backend maps and validates it.
    nora_dma_trigger_t rx_trigger;  // DMA trigger for SPIxRX
    nora_dma_trigger_t tx_trigger;  // DMA trigger for SPIxTX
    // CPU RX/TX interrupt flag and enable bits are NOT here -- see the note above.
} tdm_spi_dev_t;


//===========================================================
// Function Prototype (private)
//===========================================================
static bool hw_inst_valid( tdm_spi_inst_t inst );
static bool hw_dma_config_channel( tdm_spi_inst_t inst, nora_dma_channel_t dma_ch, nora_tdm_slot_t *buffer, uint32_t count, bool is_rx );
static void hw_spi_irq_enable( tdm_spi_inst_t inst, bool enable );
static void hw_spi_irq_disable_clear( tdm_spi_inst_t inst );
static void hw_spi_irq_bits_enable( tdm_spi_inst_t inst, bool enable );
static void hw_spi_irq_bits_clear_flags( tdm_spi_inst_t inst );


//===========================================================
// Variables
//===========================================================

// DEVICE FACTS - data sheet transcription only. Indexed by tdm_spi_inst_t.
// No interrupt-adjacent register number remains in this table: the DMA trigger is now a
// logical leg (see the note above), and the CPU RX/TX flag and enable bits live in
// hw_spi_irq_bits_*() as DFP aliases.
static const tdm_spi_dev_t s_spi_dev[TDM_SPI_INST_COUNT] =
{
    [TDM_SPI1] =
    {
        (volatile void *)&SPI1BUFL, &SPI1CON1L, &SPI1CON1H, &SPI1CON2L, &SPI1BRGL, &SPI1IMSKL,
        &SPI1STATL,
        NORA_DMA_TRIGGER_SPI1_RX, NORA_DMA_TRIGGER_SPI1_TX,
    },
    [TDM_SPI2] =
    {
        (volatile void *)&SPI2BUFL, &SPI2CON1L, &SPI2CON1H, &SPI2CON2L, &SPI2BRGL, &SPI2IMSKL,
        &SPI2STATL,
        NORA_DMA_TRIGGER_SPI2_RX, NORA_DMA_TRIGGER_SPI2_TX,
    },
#if defined(SPI3CON1L)
    [TDM_SPI3] =
    {
        (volatile void *)&SPI3BUFL, &SPI3CON1L, &SPI3CON1H, &SPI3CON2L, &SPI3BRGL, &SPI3IMSKL,
        &SPI3STATL,
        NORA_DMA_TRIGGER_SPI3_RX, NORA_DMA_TRIGGER_SPI3_TX,
    },
#else
    // No SPI3 on this device (e.g. dsPIC33CK64MC105). Left zeroed: a leg never
    // targets TDM_SPI3 unless the core's leg table puts one there, so this row
    // is dead, not just unpopulated. Zero is NORA_DMA_TRIGGER_NONE, so even if a
    // leg reached it the channel would be software-only rather than armed against
    // whatever code happened to be zero.
    [TDM_SPI3] = { 0 },
#endif
};


//===========================================================
// Global Function (silicon operations)
//===========================================================

/*
 * Program CK SPI framed-mode registers for one physical SPI instance. Writes
 * SPIxCON1L/CON1H/CON2L/BRGL from the validated config, AUDEN off (framed SPI + DMA,
 * not audio mode), leaving DMA-trigger events + the module OFF.
 */
void nora_spi_i2s_tdm_hw_apply_config( tdm_spi_inst_t inst,
                                            const nora_spi_i2s_tdm_config_t* cfg )
{
    if( !hw_inst_valid( inst ) || ( cfg == NULL ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    volatile uint16_t *con1l = dev->con1l;
    volatile uint16_t *con1h = dev->con1h;

    *con1l = 0;    // module OFF; SPIEN last
    *con1h = 0;

    // ---- CON1H: framed SPI, audio mode off ----
    nora_spi_i2s_tdm_reg_clear16(con1h, NORA_SPI_I2S_TDM_CON1H_AUDEN);   // AUDEN=0 : framed SPI, not audio mode
    nora_spi_i2s_tdm_reg_set16  (con1h, NORA_SPI_I2S_TDM_CON1H_FRMEN);   // FRMEN=1 : framed SPI (SSx = FSYNC)

    // FS waveform shape -> FRMSYPW (pulse width) + fs_words (FRMCNT cadence).
    const bool fs_50pct  = ( cfg->fs_shape == NORA_SPI_I2S_TDM_FS_50PCT );
    const bool is_i2s    = ( cfg->format   == NORA_SPI_I2S_TDM_FORMAT_I2S );
    const bool is_master = ( cfg->clock_role     == NORA_SPI_I2S_TDM_CLOCK_MASTER );
    bool    frmsypw;
    uint8_t fs_words;
    if( fs_50pct && is_i2s )
    {
        // FRMSYPW=1 = one WIRE WORD wide = 16 BCLK, which is 25% of a 64-BCLK (2 x 32-bit)
        // I2S frame, NOT 50% -- MODE16 makes the SPI marker one wire word wide. This path is therefore
        // only reachable as a SLAVE, where FS is an input and FRMSYPW describes how the
        // incoming FS is read: configure() rejects I2S + MASTER + FS_50PCT precisely so no
        // caller gets a 25% waveform in answer to a 50% request (tdm_config_is_supported).
        frmsypw  = true;
        fs_words = cfg->slots_per_fs;
    }
    else if( fs_50pct && is_master )           // FS_50PCT + TDM MASTER: half-frame marker (CLC, Stage 2)
    {
        frmsypw  = false;
        fs_words = (uint8_t)(cfg->slots_per_fs / 2u);
    }
    else                                       // FS_PULSE, or ANY slave (FS is an input)
    {
        frmsypw  = false;
        fs_words = cfg->slots_per_fs;
    }
    nora_spi_i2s_tdm_reg_set_or_clear16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMSYPW, frmsypw);

    // IGNROV + IGNTUR are HAL POLICY, not a config field (they left config_t with the
    // canonical alignment). Continuous DMA audio keeps both SET so a secondary FIFO error
    // cannot critical-stop the SPI leg and mask the primary failure; the errors are still
    // OBSERVED and counted every block (err_rov_block_count / err_tur_block_count via
    // SPIxSTATL) and RX request overrun is reported separately from DMAINTn.OVRUNIF. This is
    // containment, NOT a claim that data loss is benign. Both existing callers already
    // passed true for both, so this is byte-identical behaviour to the field version.
    nora_spi_i2s_tdm_reg_set16(con1h, NORA_SPI_I2S_TDM_CON1H_IGNROV);
    nora_spi_i2s_tdm_reg_set16(con1h, NORA_SPI_I2S_TDM_CON1H_IGNTUR);

    // FRMPOL: 1 = FS active-high (TDM), 0 = active-low (I2S).
    nora_spi_i2s_tdm_reg_set_or_clear16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMPOL,
                                     cfg->format != NORA_SPI_I2S_TDM_FORMAT_I2S);

    if( is_master )
    {
        // FRMSYNC=0 : FS output (host). MSTEN set in CON1L below.
        nora_spi_i2s_tdm_reg_clear16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMSYNC);
    }
    else
    {
        nora_spi_i2s_tdm_reg_set16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMSYNC);   // FS input (client)
    }

    // FRMCNT counts SERIAL WIRE WORDS between frame syncs, not audio slots. The serial
    // word is 16 bits (MODE16, below) while an audio slot is 32, so ONE SLOT IS TWO WIRE
    // WORDS and fs_words -- computed above in slots -- has to be scaled before it can be
    // encoded. This is why TDM8/32-bit is FRMCNT=4 (log2(16)) and not 3 (log2(8)); any
    // other value puts FS in the middle of a slot.
    const uint16_t wire_words_per_slot = (uint16_t)((cfg->word_bits + 15u) / 16u);
    const uint16_t fs_wire_words       = (uint16_t)fs_words * wire_words_per_slot;

    // FRMCNT: FS every N wire words, encoding = log2(N).
    uint16_t frmcnt;
    switch( fs_wire_words )
    {
    case 1u:  frmcnt = 0u; break;
    case 2u:  frmcnt = 1u; break;
    case 4u:  frmcnt = 2u; break;
    case 8u:  frmcnt = 3u; break;
    case 16u: frmcnt = 4u; break;
    case 32u: frmcnt = 5u; break;
    default:
        // Unreachable through configure(): tdm_config_is_supported() bounds slots_per_fs
        // so that fs_wire_words is always a power of two in 1..32. If it is reached
        // anyway, FRMEN is CLEARED rather than a plausible-looking FRMCNT written -- an
        // unframed stream is obviously broken, whereas a mis-framed one can appear to work.
        TDM_DBG_PRINTF(" ERROR: %u wire words/FS is not FRMCNT-encodable; framing OFF.\n",
                       (unsigned)fs_wire_words);
        nora_spi_i2s_tdm_reg_clear16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMEN);
        frmcnt = 0u;
        break;
    }
    nora_spi_i2s_tdm_reg_write_field16(con1h, NORA_SPI_I2S_TDM_CON1H_FRMCNT_MASK,
                                    NORA_SPI_I2S_TDM_CON1H_FRMCNT_POS, frmcnt);

    // ---- CON2L: WLENGTH must be 0 when MODE16/MODE32 select the transfer width ----
    // SPI FRM DS70005136: "when WLENGTH is N, MODE32 and MODE16 should be zero" -- the two
    // are alternative ways to set the word size, not complementary. The transfer unit here
    // is chosen by MODE16 below, so WLENGTH is 0. (It previously wrote word_bits-1 = 31
    // WHILE also setting MODE32, i.e. both mechanisms at once.)
    nora_spi_i2s_tdm_reg_write_field16(dev->con2l, NORA_SPI_I2S_TDM_CON2L_WLENGTH_MASK,
                                    NORA_SPI_I2S_TDM_CON2L_WLENGTH_POS, 0u);

    // ---- CON1L: element width, buffering, clock/role ----
    //
    // ENHBUF = 0. SPI FRM DS70005136, SPIxCON1L note 5: Standard Buffer
    // mode is the ONLY mode permitted with DMA. In Enhanced Buffer mode SPIRBF/SPITBE mean
    // "FIFO endpoint reached" rather than "one element moved", so the DMA trigger no longer
    // corresponds to one transfer.
    nora_spi_i2s_tdm_reg_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_ENHBUF);
    // MODE16 = 1 / MODE32 = 0. A 32-bit slot cannot be fed through
    // SPIxBUFL alone: the 32-bit FIFO push is triggered by the SPIxBUFH access, and no CK
    // DMA addressing mode can alternate SPIxBUFL/SPIxBUFH while also advancing through the
    // buffer (there is no stride, and one RELOAD bit reloads SRC/DST/CNT together). So the
    // WIRE unit is 16 bits and a 32-bit slot is two wire words -- which is what the DMA
    // already does (count * 2 half-word elements; see hw_dma_config_channel).
    nora_spi_i2s_tdm_reg_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_MODE32);
    nora_spi_i2s_tdm_reg_set16  (con1l, NORA_SPI_I2S_TDM_CON1L_MODE16);
    nora_spi_i2s_tdm_reg_set_or_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_MCLKEN, cfg->mclk_enable);
    nora_spi_i2s_tdm_reg_set_or_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_MSTEN, is_master);              // 1=host / 0=client
    // SPIFE: 1 = FS coincides with first BCLK (no delay), 0 = 1-bit delayed.
    nora_spi_i2s_tdm_reg_set_or_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_SPIFE, cfg->fs_coincides_first_bclk);
    nora_spi_i2s_tdm_reg_set_or_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_CKP, cfg->bclk_idle_high);
    nora_spi_i2s_tdm_reg_set_or_clear16(con1l, NORA_SPI_I2S_TDM_CON1L_CKE, cfg->bclk_change_on_active_to_idle);

    // baud rate (ignored when client / MSTEN=0). CK BRG is the low 16 bits.
    *dev->brgl = (uint16_t)(cfg->brg & 0xFFFFu);

    // Keep DMA-trigger events off until every instance is programmed; caller enables.
    nora_spi_i2s_tdm_hw_dma_trigger_enable( inst, false );

    // CPU SPI interrupts unused (DMA consumes the peripheral status flags).
    hw_spi_irq_disable_clear( inst );
}


/*
 * Configure + enable both DMA channels for one SPI instance (RX first, then TX).
 * A false return means the caller must roll back all TDM DMA/SPI state.
 */
bool nora_spi_i2s_tdm_hw_dma_config( tdm_spi_inst_t inst,
                                          nora_dma_channel_t rx_dma_ch,
                                          nora_dma_channel_t tx_dma_ch,
                                          nora_tdm_slot_t* rx_buffer,
                                          nora_tdm_slot_t* tx_buffer,
                                          uint32_t buffer_slot_count )
{
    if( !hw_inst_valid( inst ) )
    {
        return false;
    }
    if( !hw_dma_config_channel( inst, rx_dma_ch, rx_buffer, buffer_slot_count, true ) )
    {
        return false;
    }
    return hw_dma_config_channel( inst, tx_dma_ch, tx_buffer, buffer_slot_count, false );
}


/*
 * Enable/disable the SPI event->DMA triggers (SPIxIMSKL SPIRBFEN/SPITBEN), not CPU IRQs.
 */
void nora_spi_i2s_tdm_hw_dma_trigger_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    const uint16_t mask = NORA_SPI_I2S_TDM_IMSKL_SPIRBFEN | NORA_SPI_I2S_TDM_IMSKL_SPITBEN;

    nora_spi_i2s_tdm_reg_set_or_clear16( dev->imskl, mask, enable );
}


/*
 * Enable/disable one SPI module (SPIxCON1L.SPIEN only).
 */
void nora_spi_i2s_tdm_hw_module_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    nora_spi_i2s_tdm_reg_set_or_clear16( dev->con1l, NORA_SPI_I2S_TDM_CON1L_SPIEN, enable );
}


/*
 * Clear CPU RX/TX interrupt flags for one SPI instance.
 */
void nora_spi_i2s_tdm_hw_irq_clear_flags( tdm_spi_inst_t inst )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    hw_spi_irq_bits_clear_flags( inst );
}


/*
 * Soft-stop one SPI instance: disable DMA-trigger events, mask CPU SPI IRQs, clear SPIEN.
 */
void nora_spi_i2s_tdm_hw_soft_stop( tdm_spi_inst_t inst )
{
    nora_spi_i2s_tdm_hw_dma_trigger_enable( inst, false );
    hw_spi_irq_enable( inst, false );
    nora_spi_i2s_tdm_hw_module_enable( inst, false );
}


/*
 * Resolve the one register address the ISR needs from the private silicon
 * facts table.  Deliberately keep this lookup OUT of the ISR: inst_start()
 * caches it before the DMA vector can run, then the vector uses the header
 * inline helper's exact read/W0C sequence.
 */
volatile uint16_t *nora_spi_i2s_tdm_hw_get_statl( tdm_spi_inst_t inst )
{
    if( !hw_inst_valid( inst ) )
    {
        return NULL;
    }
    return s_spi_dev[inst].statl;
}


/*
 * The PPS HAL's output enum for one SPI instance's frame-sync (SSx). Master-only
 * (used by the CLC 50%-FS module in Stage 2).
 *
 * This used to hand back the raw _RPOUT_SSxOUT code, with the per-device "OUT suffix
 * or not" #ifdef ladder here. Both are the PPS HAL's business: hal_gpio/nora_pps_dspic33ck.c
 * already carries that exact ladder for NORA_PPS_OUTPUT_SSx, so returning the enum
 * instead of the code deletes the duplicate and leaves this function doing the one thing
 * only the transport HAL knows -- which SPI instance maps to which frame-sync signal.
 */
bool nora_spi_i2s_tdm_hw_get_ss_pps_output( tdm_spi_inst_t inst,
                                                 nora_pps_output_t* output )
{
    if( output == NULL )
    {
        return false;
    }
    switch( inst )
    {
    case TDM_SPI1: *output = NORA_PPS_OUTPUT_SS1; return true;
    case TDM_SPI2: *output = NORA_PPS_OUTPUT_SS2; return true;
    case TDM_SPI3: *output = NORA_PPS_OUTPUT_SS3; return true;
    default: break;
    }
    return false;
}


//===========================================================
// Local Function
//===========================================================

static bool hw_inst_valid( tdm_spi_inst_t inst )
{
    return ( (unsigned)inst < (unsigned)TDM_SPI_INST_COUNT );
}


/*
 * Configure + enable one RX or TX DMA channel for a SPI instance.
 *
 * CK DMA moves 16-bit elements; a 32-bit audio slot is two DMA half-words, so the
 * element count is buffer_slot_count * 2 and the data port is SPIxBUFL. RX copies
 * SPIxBUFL -> ping-pong buffer; TX copies buffer -> SPIxBUFL. The *2 is why the buffer
 * element is a two-uint16_t wire slot rather than an int32_t; see the wire-order
 * explanation below.
 *
 * This channel configuration is hardware-verified for 16-bit elements at SPIxBUFL, Repeated
 * One-Shot with RELOAD, HALFEN on, and CPU IRQ on RX only. Notably `DONEIF` does set at the
 * reload boundary
 * on this silicon despite DMA FRM DS30009742C §6.1, so HALF -> first half / DONE -> second
 * half is correct and no re-arm is needed.
 *
 * ---- WIRE ORDER: DMA emits the LOW half-word first ----
 *
 * The DMA walks the buffer in ascending address order and this core stores an int32_t
 * low-half-first (little-endian), which predicts the LOW 16 bits reach the wire first --
 * backwards from the MSB-first convention a TDM/I2S wire expects. **That prediction is what
 * actually happens.** An int32_t placed in these buffers goes out with its halves EXCHANGED.
 *
 * A loopback cannot see it because SDO->SDI returns whatever order it was given and RX
 * reverses the same mapping symmetrically. It matters against a real codec: every sample
 * would be half-word swapped.
 *
 * THE FIX IS IN THE BUFFER'S TYPE, not in this function. The DMA still moves 16-bit
 * elements in address order -- that is silicon, and unchangeable here. What changed is that
 * the buffers are no longer int32_t: they are nora_tdm_slot_t, whose two uint16_t
 * members are declared in WIRE order (wire[0] first). Producers convert with
 * nora_tdm_slot_encode_s32() / _decode_s32() at the point where they already store or
 * load, which measured at ~2 extra instructions per sample; `dst[i] = sample;` no longer
 * compiles. See that type's comment for the representation contract.
 *
 * Note the acceptance test is a SCOPE reading, not the loopback: with
 * DEMO_TDM_TX_PATTERN=0xFFFF0000 the burst must open with 16 BCLK HIGH straight after FS.
 * Before the fix it opened with 16 BCLK LOW. A loopback matches either way.
 *
 * Evidence, re-runnable: app/demo_tdm_master_loopback.c sends DEMO_TDM_TX_PATTERN in slots
 * 1-2 with the rest of the frame silent. With 0xFFFF_0000 (high half all ones, low half all
 * zeros) the two orders predict opposite openings:
 *
 *     high half first -> burst OPENS with 16 BCLK high, then low
 *     low  half first -> burst OPENS with 16 BCLK LOW, then high      <-- OBSERVED
 *
 * Observed at 12.5 MHz BCLK: low from the FS edge for ~1.27 us (~16 BCLK), then high 16,
 * low 16, high 16, then silence.
 *
 * It is a SWAP WITHIN EACH SLOT, not a 16-BCLK delay of the whole stream -- the two look
 * identical for one slot, so check the END: the burst finishes ~64 BCLK after the FS edge
 * (~5.17 us), exactly two slots. A global 16-BCLK delay would run to FS+80 and spill into
 * slot 3. FS-to-slot-boundary framing is therefore correct and the fault is entirely inside
 * the slot.
 *
 * MODE16 makes a 32-bit slot two independent wire words ordered by DMA memory access. The
 * hardware cannot reorder the pair: DMACHn.SIZE is one bit (byte or 16-bit word), so this
 * core has no 32-bit DMA element and MODE32 cannot be DMA-fed.
 *
 * No swap is applied in THIS function, and none should be: the fix is the buffer's element
 * type (see above and nora_tdm_slot_t). The alternatives considered and rejected
 * were a swap pass in the HAL around the block callback (3-4x the cost, since it cannot fold
 * into a store the DSP is already doing) and declaring low-half-first the contract (pushes a
 * transport detail into audio code).
 */
static bool hw_dma_config_channel( tdm_spi_inst_t inst, nora_dma_channel_t dma_ch, nora_tdm_slot_t *buffer, uint32_t count, bool is_rx )
{
    if( !hw_inst_valid( inst ) )
    {
        return false;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];

    const nora_dma_channel_cfg_t cfg =
    {
        .src   = is_rx ? dev->spi_buf : (volatile void *)buffer,
        .dst   = is_rx ? (volatile void *)buffer : dev->spi_buf,
        // 32-bit slots as 16-bit DMA half-words. Passed at full width on purpose:
        // dma_channel_config() rejects a count DMACNTn cannot hold, and truncating
        // it here would turn that rejection into a silently short transfer.
        .count = count * 2u,

        .src_mode = is_rx ? NORA_DMA_ADDR_FIXED     : NORA_DMA_ADDR_INCREMENT,
        .dst_mode = is_rx ? NORA_DMA_ADDR_INCREMENT : NORA_DMA_ADDR_FIXED,
        .size     = NORA_DMA_SIZE_HALFWORD,          // 16-bit element (CK core)
        .tr_mode  = NORA_DMA_TRMODE_REPEAT_ONESHOT,

        .reload_count = true,
        .reload_src   = !is_rx,
        .reload_dst   =  is_rx,

        .half_int_en  = true,
        .done_int_en  = true,

        .trigger      = is_rx ? dev->rx_trigger : dev->tx_trigger,

        .irq_priority_set = true,
        .irq_priority     = PRIO_TDM_DMA,
        .irq_enable       = is_rx,   // RX raises the block ISR; TX is interrupt-less
    };

    if( !nora_dma_channel_config(dma_ch, &cfg) )
    {
        TDM_DBG_PRINTF(" ERROR: DMA ch%u config failed (DMA not ready?).\n", (unsigned)dma_ch);
        return false;
    }
    if( !nora_dma_channel_enable(dma_ch, true) )
    {
        TDM_DBG_PRINTF(" ERROR: DMA ch%u enable failed (DMA not ready?).\n", (unsigned)dma_ch);
        return false;
    }
    return true;
}


static void hw_spi_irq_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    hw_spi_irq_bits_enable( inst, enable );
}


static void hw_spi_irq_disable_clear( tdm_spi_inst_t inst )
{
    hw_spi_irq_enable( inst, false );
    nora_spi_i2s_tdm_hw_irq_clear_flags( inst );
}


/*
 * The CPU-side RX/TX enable and flag bits, written one bit at a time.
 *
 * Every write below names its register and its bit at compile time and stores a literal,
 * so each one is a single bset.b / bclr.b on the shared word. The if/else is what makes
 * that true: `_SPI1RXIE = enable` hands the compiler a runtime value, and while dsPIC33C
 * has BFINS to fold that into one instruction, nothing in the C says so -- the same line
 * is a read-modify-write on dsPIC33A, where there is no BFINS.
 *
 * A part that has an SPI module whose interrupt bits the DFP does not name must fail to
 * build rather than silently ship a transport that cannot arm or clear its own interrupt.
 */
#if defined(SPI1CON1L) && !(defined(_SPI1RXIE) && defined(_SPI1TXIE) && \
                            defined(_SPI1RXIF) && defined(_SPI1TXIF))
#error "SPI1 is present but the DFP names no _SPI1RX/TXIE / _SPI1RX/TXIF bit alias"
#endif
#if defined(SPI2CON1L) && !(defined(_SPI2RXIE) && defined(_SPI2TXIE) && \
                            defined(_SPI2RXIF) && defined(_SPI2TXIF))
#error "SPI2 is present but the DFP names no _SPI2RX/TXIE / _SPI2RX/TXIF bit alias"
#endif
#if defined(SPI3CON1L) && !(defined(_SPI3RXIE) && defined(_SPI3TXIE) && \
                            defined(_SPI3RXIF) && defined(_SPI3TXIF))
#error "SPI3 is present but the DFP names no _SPI3RX/TXIE / _SPI3RX/TXIF bit alias"
#endif

static void hw_spi_irq_bits_enable( tdm_spi_inst_t inst, bool enable )
{
    if( enable )
    {
        switch( inst )
        {
        case TDM_SPI1: _SPI1RXIE = 1; _SPI1TXIE = 1; break;
        case TDM_SPI2: _SPI2RXIE = 1; _SPI2TXIE = 1; break;
#if defined(SPI3CON1L)
        case TDM_SPI3: _SPI3RXIE = 1; _SPI3TXIE = 1; break;
#endif
        default: break;
        }
    }
    else
    {
        switch( inst )
        {
        case TDM_SPI1: _SPI1RXIE = 0; _SPI1TXIE = 0; break;
        case TDM_SPI2: _SPI2RXIE = 0; _SPI2TXIE = 0; break;
#if defined(SPI3CON1L)
        case TDM_SPI3: _SPI3RXIE = 0; _SPI3TXIE = 0; break;
#endif
        default: break;
        }
    }
}


static void hw_spi_irq_bits_clear_flags( tdm_spi_inst_t inst )
{
    switch( inst )
    {
    case TDM_SPI1: _SPI1RXIF = 0; _SPI1TXIF = 0; break;
    case TDM_SPI2: _SPI2RXIF = 0; _SPI2TXIF = 0; break;
#if defined(SPI3CON1L)
    case TDM_SPI3: _SPI3RXIF = 0; _SPI3TXIF = 0; break;
#endif
    default: break;
    }
}
