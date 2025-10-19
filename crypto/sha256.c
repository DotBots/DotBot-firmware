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

void crypto_sha256_init(crypto_sha256_ctx_t *ctx) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Init(&ctx->_hash_context, CRYS_HASH_SHA256_mode);
    crypto_disable_cryptocell();
#else
    sha256_init(&ctx->_hash_context);
#endif
}

void crypto_sha256_update(crypto_sha256_ctx_t *ctx, const uint8_t *data, size_t len) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Update(&ctx->_hash_context, (uint8_t *)data, len);
    crypto_disable_cryptocell();
#else
    sha256_update(&ctx->_hash_context, (uint8_t *)data, len);
#endif
}

void crypto_sha256(crypto_sha256_ctx_t *ctx, uint8_t *digest) {
#if defined(USE_CRYPTOCELL)
    crypto_enable_cryptocell();
    CRYS_HASH_Finish(&ctx->_hash_context, (uint32_t *)digest);
    crypto_disable_cryptocell();
#else
    sha256_final(&ctx->_hash_context, (BYTE *)digest);
#endif
}
