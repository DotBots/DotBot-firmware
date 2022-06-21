#ifndef __US_RANGING_H
#define __US_RANGING_H

/**
 * @file us_ranging.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "us_ranging" bsp module.
 *
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
 

//=========================== defines ======================================

typedef void(*us_callback_t)(uint32_t);
typedef void(*timer_callback_t)(void);

//=========================== public ======================================
 
void hc_sr04_init(us_callback_t us_callback, timer_callback_t timer_callback, NRF_TIMER_Type *us_on, NRF_TIMER_Type *us_read);
void hc_sr04_start(void);
void hc_sr04_stop(void);
void hc_sr04_on_set_trigger(double duration_ms, double offset_ms);

void hfclk_init(void);

#endif