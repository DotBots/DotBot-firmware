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

#include "cc3xx_psa_asymmetric_signature.h"

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

    // trying to instantiate attributes for cc3xx_sign_message...
    //psa_key_attributes_t *attributes = psa_key_attributes_init();
    //cc3xx_sign_message(attributes, NULL, 0, NULL, NULL, 0, NULL, 0, NULL);

    // the real limitation is here: how to overcome "undefined symbol psa_driver_wrapper_hash_compute" and friends
    cc3xx_sign_message(NULL, NULL, 0, NULL, NULL, 0, NULL, 0, NULL);

    puts("SUCCESS!");

    while (1) {
        __WFE();
    }
}
