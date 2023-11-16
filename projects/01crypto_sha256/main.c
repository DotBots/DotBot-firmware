/**
 * @file 01crypto_hash.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the GPIO api.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sha256.h"

#include "expected_sha256.h"

//=========================== defines ==========================================

#define HASH_LEN (32U)

//=========================== variables ========================================

static const char *hash_input            = "HashThis!";
static uint8_t     hash_result[HASH_LEN] = { 0 };

//=========================== main =============================================

int main(void) {
    printf("Testing sha256 hash: ");
    crypto_sha256_init();
    crypto_sha256_update((const uint8_t *)hash_input, strlen(hash_input));
    crypto_sha256(hash_result);

    if (strncmp((const char *)hash_result, (const char *)expected_sha256, HASH_LEN)) {
        puts("FAILURE!");
        return 1;
    }
    puts("SUCCESS!");

    while (1) {
        __WFE();
    }
}
