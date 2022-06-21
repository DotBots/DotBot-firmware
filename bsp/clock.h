#ifndef __CLOCK_H
#define __CLOCK_H

/**
 * @file clock.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "clock" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

void db_hfclk_init(void);
void db_hfclk_deinit(void);
void db_lfclk_init(void);
void db_lfclk_deinit(void);

#endif
