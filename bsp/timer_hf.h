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

typedef void (*timer_hf_cb_t)(void);

//=========================== prototypes =======================================

/**
 * @brief Configure a high frequency timer with microsecond precision
 */
void db_timer_hf_init(void);

/**
 * @brief Return the current timer time in microseconds
 */
uint32_t db_timer_hf_now(void);

/**
 * @brief Set a callback to be called periodically using the high frequency timer
 *
 * @param[in] channel   TIMER channel used
 * @param[in] us        periodicity in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_periodic_us(uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of microseconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] us        delay in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_us(uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_ms(uint8_t channel, uint32_t ms, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called after an amount of seconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_s(uint8_t channel, uint32_t s, timer_hf_cb_t cb);

/**
 * @brief Add a delay in us using the high frequency timer
 *
 * @param[in] us    delay in us
 */
void db_timer_hf_delay_us(uint32_t us);

/**
 * @brief Add a delay in milliseconds using the high frequency timer
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_hf_delay_ms(uint32_t ms);

/**
 * @brief Add a delay in seconds using the high frequency timer
 *
 * @param[in] s delay in seconds
 */
void db_timer_hf_delay_s(uint32_t s);

#endif
