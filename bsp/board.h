#ifndef __BOARD_H
#define __BOARD_H

/**
 * @defgroup    bsp_board   Board
 * @ingroup     bsp
 * @brief       Cross-platform declaration "board" bsp module
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

/**
 * @brief Turn ON the DotBot board
 *
 * Especifically turn on the board regulator if there's one
 */
void db_board_init(void);

#endif
