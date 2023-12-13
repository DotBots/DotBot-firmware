/**
 * @file
 * @ingroup samples_crypto
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 * @brief Maybe cc312 will work.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "ed25519.h"

#include "private_key.h"
#include "public_key.h"
#include "expected_signature.h"

//=========================== defines ==========================================

#define ED25519_PRIVATE_KEY_LEN (32U)
#define ED25519_PUBLIC_KEY_LEN  (32U)
#define ED25519_SIGNATURE_LEN   (64U)

//=========================== variables ========================================

//static const char *signature_input                         = "SignThis!";
//static uint8_t     signature_result[ED25519_SIGNATURE_LEN] = { 0 };

//=========================== main =============================================

int main(void) {

    puts("SUCCESS!");

    while (1) {
        __WFE();
    }
}
