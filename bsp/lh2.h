#ifndef LH2_H_
#define LH2_H_

/**
 * @file lh2.h
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "lh2" bsp module.
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>, Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== public ==========================================

// initialization function
void db_lh2_init(void);

// do fil's stuff
bool db_get_black_magic(void);

// function for the dotbot to get hold of the current location packet when it is ready
void db_get_current_location(uint32_t *location);

// function to restart SPM3
void db_lh2_start_transfer(void);

//=========================== private ==========================================

// these functions are called in the order written to perform the LH2 localization
void db_lh2_initialize_ts4231(void);

uint64_t db_lh2_demodulate_light(uint8_t *sample_buffer);

uint64_t poly_check(uint32_t poly, uint32_t bits, uint8_t numbits);
int      LH2_determine_polynomial(uint64_t chipsH1, int *start_val);

uint64_t hamming_weight(uint64_t bits_in);

uint32_t reverse_count_p0(uint32_t bits);
uint32_t reverse_count_p1(uint32_t bits);
uint32_t reverse_count_p2(uint32_t bits);
uint32_t reverse_count_p3(uint32_t bits);

// setup the PPI
void ppi_setup(void);
void timer2_setup(void);
void gpiote_setup(void);

// Said Set-Up
void lh2_pin_set_input(uint8_t pin);
void lh2_pin_set_output(uint8_t pin);

#endif /* LH2_H_ */
