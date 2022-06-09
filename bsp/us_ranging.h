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
 
 //=========================== public ======================================
 
void us_init(void (*callback)(uint32_t), NRF_TIMER_Type *us_on, NRF_TIMER_Type *us_read);
void us_on_set_trigger(double duration_ms, double period_ms);
void us_start(void);



 #endif