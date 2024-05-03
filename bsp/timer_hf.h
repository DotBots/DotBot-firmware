#ifndef __TIMER_HF_H
#define __TIMER_HF_H

/**
 * @defgroup    bsp_timer_hf    High Frequency Timer
 * @ingroup     bsp
 * @brief       High level timing functions on top of the TIMER peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>

//=========================== defines ==========================================

typedef uint8_t timer_hf_t;  ///< TIMER peripheral index

typedef void (*timer_hf_cb_t)(void);  ///< Callback function prototype, it is called when the timer fires an event

//=========================== prototypes =======================================

/**
 * @brief Configure a high frequency timer with microsecond precision
 *
 * @param[in] timer     timer reference used
 */
void db_timer_hf_init(timer_hf_t timer);

/**
 * @brief Return the current timer time in microseconds
 *
 * @param[in] timer     timer reference used
 */
uint32_t db_timer_hf_now(timer_hf_t timer);

/**
 * @brief Set a callback to be called periodically using the high frequency timer
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   TIMER channel used
 * @param[in] us        periodicity in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_periodic_us(timer_hf_t timer, uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of microseconds
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   TIMER channel used
 * @param[in] us        delay in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_us(timer_hf_t timer, uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   TIMER channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_ms(timer_hf_t timer, uint8_t channel, uint32_t ms, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called after an amount of seconds
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   TIMER channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_s(timer_hf_t timer, uint8_t channel, uint32_t s, timer_hf_cb_t cb);

/**
 * @brief Add a delay in us using the high frequency timer
 *
 * @param[in] timer     timer reference used
 * @param[in] us        delay in us
 */
void db_timer_hf_delay_us(timer_hf_t timer, uint32_t us);

/**
 * @brief Add a delay in milliseconds using the high frequency timer
 *
 * @param[in] timer     timer reference used
 * @param[in] ms        delay in milliseconds
 */
void db_timer_hf_delay_ms(timer_hf_t timer, uint32_t ms);

/**
 * @brief Add a delay in seconds using the high frequency timer
 *
 * @param[in] timer     timer reference used
 * @param[in] s         delay in seconds
 */
void db_timer_hf_delay_s(timer_hf_t timer, uint32_t s);

#endif
