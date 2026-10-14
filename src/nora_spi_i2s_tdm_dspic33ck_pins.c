/*
 * nora_spi_i2s_tdm_dspic33ck_pins.c -- SPI1 TDM pin routing, shared by every board.
 *
 * See the header for why this is here rather than in each board. This file is the
 * merge of ev88g73a_tdm_pins_init() and dm330030_tdm_pins_init(), which differed
 * only in their four RP numbers.
 *
 * THE ORDER IS EV88G73A'S, DELIBERATELY
 * -------------------------------------
 * Directions first for all four pins, PPS afterwards for all four -- not
 * direction+route pin by pin. Outputs are seeded low before PPS starts driving
 * anything, and this two-phase order is the one that was scope-verified on
 * EV88G73A with the loopback demo. DM330030's SLAVE branch interleaved the two
 * phases instead; it converges onto the verified order here, which is a change
 * only on the board that has no hardware yet.
 */

#include "nora_spi_i2s_tdm_dspic33ck_pins.h"

#include <stddef.h>

#include "nora_pps.h"

bool nora_spi_i2s_tdm_pins_configure(const nora_spi_i2s_tdm_pinmap_t *map,
                                          nora_spi_i2s_tdm_clock_role_t          role)
{
    if (map == NULL) {
        return false;
    }

    /* The dsPIC drives the clocks only as master. SDO is always ours, SDI never is. */
    const bool clocks_are_ours = (role == NORA_SPI_I2S_TDM_CLOCK_MASTER);

    /* Phase 1 -- directions, outputs seeded low, before any PPS drives a pad. */
    if (clocks_are_ours) {
        if (!nora_gpio_rp_config_digital_output(map->bclk, false)) return false;
        if (!nora_gpio_rp_config_digital_output(map->fs,   false)) return false;
    } else {
        if (!nora_gpio_rp_config_digital_input(map->bclk)) return false;
        if (!nora_gpio_rp_config_digital_input(map->fs))   return false;
    }
    if (!nora_gpio_rp_config_digital_output(map->sdo, false)) return false;
    if (!nora_gpio_rp_config_digital_input (map->sdi))        return false;

    /* Phase 2 -- PPS. Same four tokens either way; only the direction differs. */
    if (clocks_are_ours) {
        if (!nora_pps_route_output(NORA_PPS_OUTPUT_SCK1, map->bclk)) return false;
        if (!nora_pps_route_output(NORA_PPS_OUTPUT_SS1,  map->fs))   return false;
    } else {
        if (!nora_pps_route_input(NORA_PPS_INPUT_SCK1, map->bclk)) return false;
        if (!nora_pps_route_input(NORA_PPS_INPUT_SS1,  map->fs))   return false;
    }
    if (!nora_pps_route_output(NORA_PPS_OUTPUT_SDO1, map->sdo)) return false;
    if (!nora_pps_route_input (NORA_PPS_INPUT_SDI1,  map->sdi)) return false;

    return true;
}
