/**
 * @file
 * @ingroup crypto
 *
 * @brief  SHA256 hashing implementation using CryptoCell.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>

#include <nrf.h>
#include "sha256.h"
#include "utils.h"

#if defined(USE_CRYPTOCELL)
#include "nrf_cc310/include/crys_hash.h"

static CRYS_HASHUserContext_t _hash_context;
#else
#include "soft_sha256.h"

static SHA256_CTX _hash_context;
#endif

void crypto_sha256_init(void) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Init(&_hash_context, CRYS_HASH_SHA256_mode);
    crypto_disable_cryptocell();
#else
    sha256_init(&_hash_context);
#endif
}

void crypto_sha256_update(const uint8_t *data, size_t len) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Update(&_hash_context, (uint8_t *)data, len);
    crypto_disable_cryptocell();
#else
    sha256_update(&_hash_context, (uint8_t *)data, len);
#endif
}

void crypto_sha256(uint8_t *digest) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Finish(&_hash_context, (uint32_t *)digest);
    crypto_disable_cryptocell();
#else
    sha256_final(&_hash_context, (BYTE *)digest);
#endif
}
