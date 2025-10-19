/**
 * @file
 * @ingroup samples_crypto
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the GPIO api.
 *
 * @copyright Inria, 2023
 *
 */
#include <stdio.h>
#include <string.h>
#include <nrf.h>
#include "sha256.h"

#include "expected_sha256.h"

//=========================== defines ==========================================

#define HASH_LEN (32U)

//=========================== variables ========================================

static const char         *hash_input            = "HashThis!";
static uint8_t             hash_result[HASH_LEN] = { 0 };
static crypto_sha256_ctx_t sha256_ctx            = { 0 };

//=========================== main =============================================

int main(void) {
    printf("Testing sha256 hash: ");
    crypto_sha256_init(&sha256_ctx);
    crypto_sha256_update(&sha256_ctx, (const uint8_t *)hash_input, strlen(hash_input));
    crypto_sha256(&sha256_ctx, hash_result);

    if (strncmp((const char *)hash_result, (const char *)expected_sha256, HASH_LEN)) {
        puts("FAILURE!");
        return 1;
    }
    puts("SUCCESS!");

    while (1) {
        __WFE();
    }
}
