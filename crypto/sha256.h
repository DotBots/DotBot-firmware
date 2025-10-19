#ifndef __SHA256_H
#define __SHA256_H

/**
 * @defgroup    crypto_sha256   SHA256 hashing support
 * @ingroup     crypto
 * @brief       Hash data using SHA256
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdlib.h>
#include <stdint.h>

#if defined(USE_CRYPTOCELL)
#include "utils.h"
#include "nrf_cc310/include/crys_hash.h"
#else
#include "soft_sha256.h"
#endif

typedef struct {
#if defined(USE_CRYPTOCELL)
    CRYS_HASHUserContext_t _hash_context;
#else
    SHA256_CTX _hash_context;
#endif
} crypto_sha256_ctx_t;

/**
 * @brief   Initialize the SHA256 hashing process
 *
 * @param[in]   ctx             Context of the SHA256 hashing process
 */
void crypto_sha256_init(crypto_sha256_ctx_t *ctx);

/**
 * @brief   Add new data to the running hash process
 *
 * @param[in]   ctx             Context of the SHA256 hashing process
 * @param[in]   data            Input data
 * @param[in]   len             Input data length
 */
void crypto_sha256_update(crypto_sha256_ctx_t *ctx, const uint8_t *data, size_t len);

/**
 * @brief   Returns the computed hash
 *
 * @param[in]   ctx             Context of the SHA256 hashing process
 * @param[in]   digest          Computed hash
 */
void crypto_sha256(crypto_sha256_ctx_t *ctx, uint8_t *digest);

#endif  // __SHA256_H
