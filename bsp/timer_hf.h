#ifndef __TIMER_H
#define __TIMER_H

/**
 * @file timer.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "timer hf" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

typedef void(*timer_hf_cb_t)(void);

//=========================== prototypes =======================================

void db_timer_hf_init(void);
void db_timer_hf_set_periodic(uint8_t channel, uint32_t us, timer_hf_cb_t cb);
void db_timer_hf_set_callback_us(uint8_t channel, uint32_t us, timer_hf_cb_t cb);
void db_timer_hf_set_callback_ms(uint8_t channel, uint32_t ms, timer_hf_cb_t cb);
void db_timer_hf_set_callback_s(uint8_t channel, uint32_t s, timer_hf_cb_t cb);
void db_timer_hf_delay_us(uint32_t us);
void db_timer_hf_delay_ms(uint32_t ms);
void db_timer_hf_delay_s(uint32_t s);

#endif
