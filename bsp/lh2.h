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
void lh2_init(void);

// do fil's stuff
bool get_black_magic(void);

// test function
bool get_black_magic_2(void);

// function for the dotbot to get hold of the current location packet when it is ready
uint32_t *get_current_location(void);

// function to restart SPM3
void start_transfer(void);

void restart_lh2(void);

//=========================== private ==========================================

// these functions are called in the order written to perform the LH2 localization
void LH2_initialize_TS4231(void);

uint64_t LH2_demodulate_light(uint8_t *sample_buffer);

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
void lh2_wait_timer_config(void);
void lh2_wait_usec(uint32_t usec);

#endif /* LH2_H_ */