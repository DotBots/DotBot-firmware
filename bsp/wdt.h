#ifndef __WDT_H
#define __WDT_H

/**
 * @defgroup    bsp_wdt Watchdog timer
 * @ingroup     bsp
 * @brief       Watchdog timer driver implementation
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 * @}
 */

#include <stdint.h>
#include <nrf.h>

typedef void (*wdt_timeout_cb_t)(void *ctx);  ///< Timeout callback function prototype

/**
 * @brief Initialize the watchdog timer
 *
 * @param[in] timeout   Timeout in seconds
 * @param[in] cb        Callback function called on timeout
 * @param[in] ctx       Context passed to the callback function
 */
void db_wdt_init(uint32_t timeout, wdt_timeout_cb_t cb, void *ctx);

/**
 * @brief Start the watchdog timer
 */
void db_wdt_start(void);

/**
 * @brief Reload the watchdog timer
 */
void db_wdt_reload(void);

#endif
