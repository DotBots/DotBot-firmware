#ifndef __SHA256_H
#define __SHA256_H

/**
 * @defgroup    crypto_sha256   SHA256 hashing support
 * @ingroup     crypto
 * @brief       Hash data using SHA256
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
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
 * @param[in]   data            Input data
 * @param[in]   len             Input data length
 */
void crypto_sha256_update(const uint8_t *data, size_t len);

/**
 * @brief   Returns the computed hash
 *
 * @param[in]   digest          Computed hash
 */
void crypto_sha256(uint8_t *digest);

#endif  // __SHA256_H
