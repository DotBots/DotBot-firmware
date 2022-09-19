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
bool db_lh2_get_black_magic(void);

// function for the dotbot to get hold of the current location packet when it is ready
void db_lh2_get_current_location(uint32_t *location);

// function to restart SPM3
void db_lh2_start_transfer(void);

#endif /* LH2_H_ */
