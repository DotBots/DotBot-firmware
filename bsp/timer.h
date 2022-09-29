#ifndef __TIMER_H
#define __TIMER_H

/**
 * @file timer.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "timer" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

typedef void (*timer_cb_t)(void);

//=========================== prototypes =======================================

/**
 * @brief Configure an RTC timer with millisecond precision
 */
void db_timer_init(void);

/**
 * @brief Returns the number of ticks since timer initialization
 *
 * @return number of ticks (1 tick ~ 30us)
 */
uint32_t db_timer_ticks(void);

/**
 * @brief Set a callback to be called periodically
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        periodicity in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_periodic_ms(uint8_t channel, uint32_t ms, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of ticks (1 tick ~= 30us)
 *
 * @param[in] channel   RTC channel used
 * @param[in] ticks     delay in ticks
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ticks(uint8_t channel, uint32_t ticks, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ms(uint8_t channel, uint32_t ms, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of seconds
 *
 * @param[in] channel   RTC channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_s(uint8_t channel, uint32_t s, timer_cb_t cb);

/**
 * @brief Add a delay in ticks (1 tick ~ 30us)
 *
 * @param[in] ticks delay in ticks
 */
void db_timer_delay_ticks(uint32_t ticks);

/**
 * @brief Add a delay in milliseconds
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_delay_ms(uint32_t ms);

/**
 * @brief Add a delay in seconds
 *
 * @param[in] s delay in seconds
 */
void db_timer_delay_s(uint32_t s);

#endif
