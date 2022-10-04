#ifndef __DEVICE_H
#define __DEVICE_H

/**
 * @file device.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "device" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <nrf.h>

/**
 * @brief Returns the 32bit device address (32 bits)
 *
 * @return device address in 32bit format
 */
static inline uint32_t db_device_addr(void) {
    return *(uint32_t *)&NRF_FICR->DEVICEADDR[0];
}

/**
 * @brief Fetch the unique device identifier (64 bits)
 *
 * @return device identifier in 64bit format
 */
static inline uint64_t db_device_id(void) {
    return ((uint64_t)(*(uint32_t *)&NRF_FICR->DEVICEADDR[1]) << 32) | *(uint32_t *)&NRF_FICR->DEVICEADDR[0];
}

#endif
