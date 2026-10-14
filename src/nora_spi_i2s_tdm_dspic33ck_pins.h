#ifndef NORA_SPI_I2S_TDM_DSPIC33CK_PINS_H
#define NORA_SPI_I2S_TDM_DSPIC33CK_PINS_H

/*
 * SPI1 pin routing for the TDM/I2S transport, once, for every board.
 *
 * WHY THIS IS IN THE HAL AND NOT IN A BOARD
 * -----------------------------------------
 * ev88g73a_tdm_pins_init() and dm330030_tdm_pins_init() were the same forty lines
 * twice: the same four directions, the same four PPS tokens, the same role switch.
 * The only difference between them was four RP numbers -- and an RP number is a
 * board fact, while "SPI1's frame clock is the SS1 token and it is an output when
 * we are the master" is a fact about the device family.
 *
 * So the family fact moves here and each board is left with its four numbers, in
 * the same spirit as nora_spi_i2s_tdm_dspic33ck_fs_clc.c already owning CLC1 so that no
 * board has to know CLC exists.
 *
 * A board still owns the CALL: it decides that these are its TDM pins and passes
 * the map. Nothing here reads a board header.
 */

#include <stdbool.h>

#include "nora_gpio.h"
#include "nora_spi_i2s_tdm.h"

/*
 * One board's four TDM pins, as RP numbers.
 *
 * FS is the pin the SPI's FRMSYNC (SS1) output goes to. When the transport is
 * asked for a 50%-duty FS it repoints that pad at CLC1OUT itself, by
 * reverse-scanning RPOR -- so this stays four plain numbers and the board still
 * knows nothing about CLC1.
 */
typedef struct {
    nora_gpio_rp_t bclk;   /* SCK1 */
    nora_gpio_rp_t fs;     /* SS1 (FRMSYNC) */
    nora_gpio_rp_t sdo;    /* SDO1, dsPIC -> codec */
    nora_gpio_rp_t sdi;    /* SDI1, codec -> dsPIC */
} nora_spi_i2s_tdm_pinmap_t;

/*
 * Route those four pins for the given role.
 *
 *   MASTER: the dsPIC drives BCLK, FS and SDO; SDI is an input.
 *   SLAVE:  the codec drives BCLK and FS, so SDO is the dsPIC's only output.
 *
 * The role is an argument and not a build-time switch because the wiring supports
 * both directions on both boards: which side drives BCLK/FS is the profile's
 * choice, not the board's.
 *
 * Returns false on the first GPIO or PPS call that refuses, leaving the pins
 * partly routed -- the same behaviour the two board functions had.
 */
bool nora_spi_i2s_tdm_pins_configure(const nora_spi_i2s_tdm_pinmap_t *map,
                                          nora_spi_i2s_tdm_clock_role_t          role);

#endif /* NORA_SPI_I2S_TDM_DSPIC33CK_PINS_H */
