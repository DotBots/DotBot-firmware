#ifndef __CRYPTO_UTILS_H
#define __CRYPTO_UTILS_H

/**
 * @file utils.h
 * @addtogroup Crypto
 *
 * @brief  Declaration for crypto module
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <nrf.h>

//=========================== defines ==========================================

#if defined(NRF5340_XXAA_APPLICATION)
#define NRF_CRYPTOCELL NRF_CRYPTOCELL_S
#endif

/**
 * @brief   Enables the CryptoCell
 *
 * Must be called before each call to the CryptoCell functions.
 */
static inline void crypto_enable_cryptocell(void) {
    NRF_CRYPTOCELL->ENABLE = CRYPTOCELL_ENABLE_ENABLE_Enabled << CRYPTOCELL_ENABLE_ENABLE_Pos;
}

/**
 * @brief   Disables the CryptoCell
 *
 * Must be called after each call to the CryptoCell functions.
 */
static inline void crypto_disable_cryptocell(void) {
    NRF_CRYPTOCELL->ENABLE = CRYPTOCELL_ENABLE_ENABLE_Disabled << CRYPTOCELL_ENABLE_ENABLE_Pos;
}

#endif  // __CRYPTO_UTILS_H
