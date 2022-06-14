#ifndef __RPM_H
#define __RPM_H

/**
 * @file rpm.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "rpm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== prototypes ======================================

//=========================== public ======================================

void db_rpm_init(void);
void db_rpm_encoder_timers_start(void);
void db_rpm_encoder_timers_stop(void);
uint32_t db_rpm_get_left_speed(void);
uint32_t db_rpm_get_right_speed(void);
uint32_t db_rpm_get_left_rpm(void);
uint32_t db_rpm_get_right_rpm(void);
uint32_t db_rpm_get_left_rps(void);
uint32_t db_rpm_get_right_rps(void);

#endif
