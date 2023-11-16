#ifndef __SHA256_H
#define __SHA256_H

/**
 * @file crypto.h
 * @addtogroup Crypto
 *
 * @brief  Declaration for crypto module
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdlib.h>
#include <stdint.h>

/**
 * @brief   Initialize the SHA256 hashing process
 */
void crypto_sha256_init(void);

/**
 * @brief   Add new data to the running hash process
 *
 * @param[input]    data            Input data
 * @param[input]    len             Input data length
 */
void crypto_sha256_update(const uint8_t *data, size_t len);

/**
 * @brief   Returns the computed hash
 *
 * @param[output]   digest          Computed hash
 */
void crypto_sha256(uint8_t *digest);

#endif  // __SHA256_H
