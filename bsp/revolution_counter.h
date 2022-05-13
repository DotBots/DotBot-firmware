#ifndef __REVOLUTION_COUNTER_H
#define __REVOLUTION_COUNTER_H

/**
 * @file revolution_counter.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "revolution counter" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== prototypes ======================================

//=========================== public ======================================

void db_revolution_counter_init();
void db_board_encoder_timers_start(void);
void db_board_encoder_timers_stop(void);
float db_board_get_left_speed(void);
float db_board_get_right_speed(void);
uint32_t db_board_get_left_rpm(void);
uint32_t db_board_get_right_rpm(void);

#endif
