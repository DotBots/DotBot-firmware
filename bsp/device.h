#ifndef __DEVICE_H
#define __DEVICE_H

/**
 * @defgroup    bsp_device  Device information
 * @ingroup     bsp
 * @brief       Provides functions to retrieve device unique identifier and address
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <nrf.h>

#if defined(USE_SWARMIT)
uint64_t swarmit_read_device_id(void);
#endif

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define NRF_FICR NRF_FICR_NS
#endif

/**
 * @brief Returns the 32bit device address (32 bits)
 *
 * @return device address in 32bit format
 */
static inline uint64_t db_device_addr(void) {
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
    return 0;
#else
    return ((((uint64_t)NRF_FICR->DEVICEADDR[1]) << 32) & 0xffffffffffff) | (uint64_t)NRF_FICR->DEVICEADDR[0];
#endif
}

/**
 * @brief Fetch the unique device identifier (64 bits)
 *
 * @return device identifier in 64bit format
 */
static inline uint64_t db_device_id(void) {
#if defined(NRF5340_XXAA)
#if defined(NRF_NETWORK)
    return ((uint64_t)NRF_FICR_NS->INFO.DEVICEID[1]) << 32 | (uint64_t)NRF_FICR_NS->INFO.DEVICEID[0];
#elif defined(USE_SWARMIT)
    return swarmit_read_device_id();
#elif defined(NRF_APPLICATION)
    return ((uint64_t)NRF_FICR_S->INFO.DEVICEID[1]) << 32 | (uint64_t)NRF_FICR_S->INFO.DEVICEID[0];
#endif
#else
    return ((uint64_t)NRF_FICR->DEVICEID[1]) << 32 | (uint64_t)NRF_FICR->DEVICEID[0];
#endif
}

#endif
