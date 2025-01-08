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
#include "ed25519.h"

#include "private_key.h"
#include "public_key.h"
#include "expected_signature.h"

//=========================== defines ==========================================

#define ED25519_PRIVATE_KEY_LEN (32U)
#define ED25519_PUBLIC_KEY_LEN  (32U)
#define ED25519_SIGNATURE_LEN   (64U)

//=========================== variables ========================================

static const char *signature_input                         = "SignThis!";
static uint8_t     signature_result[ED25519_SIGNATURE_LEN] = { 0 };

//=========================== main =============================================

int main(void) {
    printf("Testing ed25519 signature: ");
    size_t signature_len = crypto_ed25519_sign(signature_result, (const uint8_t *)signature_input, strlen(signature_input), private_key, public_key);
    if (signature_len != ED25519_SIGNATURE_LEN) {
        puts("FAILURE!");
        return 1;
    }

    if (strncmp((const char *)signature_result, (const char *)expected_signature, ED25519_SIGNATURE_LEN)) {
        puts("FAILURE!");
        return 1;
    }

    bool verify = crypto_ed25519_verify(signature_result, signature_len, (const uint8_t *)signature_input, strlen(signature_input), public_key);
    if (!verify) {
        puts("FAILURE!");
        return 1;
    }

    puts("SUCCESS!");

    while (1) {
        __WFE();
    }
}
