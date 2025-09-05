#ifndef __TIMER_H
#define __TIMER_H

/**
 * @defgroup    bsp_timer   Low Frequency Timer
 * @ingroup     bsp
 * @brief       High level timing functions on top of the RTC peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>

//=========================== defines ==========================================

typedef uint8_t timer_t;  ///< RTC peripheral index

typedef void (*timer_cb_t)(void);  ///< Callback function prototype, it is called when the timer fires an event

//=========================== prototypes =======================================

/**
 * @brief Configure an RTC timer with millisecond precision
 *
 * @param[in] timer     timer reference used
 */
void db_timer_init(timer_t timer);

/**
 * @brief Start an RTC timer
 *
 * @param[in] timer     timer reference used
 */
void db_timer_start(timer_t timer);

/**
 * @brief Stop an RTC timer
 *
 * @param[in] timer     timer reference used
 */
void db_timer_stop(timer_t timer);

/**
 * @brief Returns the number of ticks since timer initialization
 *
 * @param[in] timer     timer reference used
 *
 * @return number of ticks (1 tick ~ 30us)
 */
uint32_t db_timer_ticks(timer_t timer);

/**
 * @brief Set a callback to be called periodically
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   RTC channel used
 * @param[in] ms        periodicity in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_periodic_ms(timer_t timer, uint8_t channel, uint32_t ms, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of ticks (1 tick ~= 30us)
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   RTC channel used
 * @param[in] ticks     delay in ticks
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ticks(timer_t timer, uint8_t channel, uint32_t ticks, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   RTC channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ms(timer_t timer, uint8_t channel, uint32_t ms, timer_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of seconds
 *
 * @param[in] timer     timer reference used
 * @param[in] channel   RTC channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_s(timer_t timer, uint8_t channel, uint32_t s, timer_cb_t cb);

/**
 * @brief Add a delay in ticks (1 tick ~ 30us)
 *
 * @param[in] timer     timer reference used
 * @param[in] ticks delay in ticks
 */
void db_timer_delay_ticks(timer_t timer, uint32_t ticks);

/**
 * @brief Add a delay in milliseconds
 *
 * @param[in] timer     timer reference used
 * @param[in] ms    delay in milliseconds
 */
void db_timer_delay_ms(timer_t timer, uint32_t ms);

/**
 * @brief Add a delay in seconds
 *
 * @param[in] timer     timer reference used
 * @param[in] s delay in seconds
 */
void db_timer_delay_s(timer_t timer, uint32_t s);

#endif
