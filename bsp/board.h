#ifndef __BOARD_H
#define __BOARD_H

/**
 * @file board.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "board" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

/**
 * @brief Turn ON the DotBot board
 *
 * Especifically turn on the Board Regulator
 * all the on board regulators ENABLE pins
 * are tied to the nRF pin P0.20
 */
void db_board_init(void);

#endif
