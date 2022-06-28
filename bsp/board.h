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

//=========================== public ===========================================

void db_board_init(void);
void db_board_regulator_on(void);
void db_board_regulator_off(void);

#endif
