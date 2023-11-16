#include <stdint.h>

#include <nrf.h>
#include "sha256.h"
#include "utils.h"

#include "nrf_cc310/include/crys_hash.h"

static CRYS_HASHUserContext_t _hash_context;

void crypto_sha256_init(void) {
    crypto_enable_cryptocell();
    CRYS_HASH_Init(&_hash_context, CRYS_HASH_SHA256_mode);
    crypto_disable_cryptocell();
}

void crypto_sha256_update(const uint8_t *data, size_t len) {
    crypto_enable_cryptocell();
    CRYS_HASH_Update(&_hash_context, (uint8_t *)data, len);
    crypto_disable_cryptocell();
}

void crypto_sha256(uint8_t *digest) {
    crypto_enable_cryptocell();
    CRYS_HASH_Finish(&_hash_context, (uint32_t *)digest);
    crypto_disable_cryptocell();
}
