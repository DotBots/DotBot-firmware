#ifndef __ED25519_H
#define __ED25519_H

/**
 * @defgroup    crypto_ed25519  Ed25519 signature support
 * @ingroup     crypto
 * @brief       Sign/verify data using Ed25519
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   Compute the ED25519 signature of a data
 *
 * @param[out]  signature       Computed signature
 * @param[in]   data            Input data
 * @param[in]   data_len        Input data length
 * @param[in]   private_key     Private key used to sign the data, must be 32 Bytes
 * @param[in]   public_key      Public key used to sign the data, must be 32 Bytes
 *
 * @return the size of the computed signature, must be 64
 */
size_t crypto_ed25519_sign(uint8_t *signature, const uint8_t *data, size_t data_len, const uint8_t *private_key, const uint8_t *public_key);

/**
 * @brief   Verify the ED25519 signature of a data
 *
 * @param[in]   signature       Signature
 * @param[in]   signature_len   Signature length
 * @param[in]   data            Input data
 * @param[in]   data_len        Input data length
 * @param[in]   public_key      Public key used to verify the data, must be 32 Bytes
 *
 * @return true if the signature could be verified, false otherwise
 */
bool crypto_ed25519_verify(const uint8_t *signature, size_t signature_len, const uint8_t *data, size_t data_len, const uint8_t *public_key);

#endif  // __ED25519_H
