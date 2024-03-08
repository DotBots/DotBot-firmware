#ifndef __UPGATE_H
#define __UPGATE_H

/**
 * @defgroup    drv_upgate  FPGA bitstream update library
 * @ingroup     drv
 * @brief       FPGA bitstream update library
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024-present
 * @}
 */

#include <stdint.h>
#include "n25q128.h"

//=========================== defines ==========================================

#define DB_UPGATE_CHUNK_SIZE       (128U)  ///< Size of an FPGA bitstream chunk
#define DB_UPGATE_SHA256_LENGTH    (32U)
#define DB_UPGATE_SIGNATURE_LENGTH (64U)

typedef void (*db_upgate_reply_t)(const uint8_t *, size_t);  ///< Transport agnostic function used to reply to the flasher script

///< FPGA bitstream update configuration
typedef struct {
    db_upgate_reply_t     reply;  ///< Pointer to the function used to reply to the upgate script
    const n25q128_conf_t *n25q128_conf;
} db_upgate_conf_t;

///< FPGA bitstream update start notification packet
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t chunk_count;      ///< Number of chunks
    bool     use_compression;  ///< True if bitstream is compressed
#if defined(UPGATE_USE_CRYPTO)
    uint8_t hash[DB_UPGATE_SHA256_LENGTH];          ///< SHA256 hash of the bitsream
    uint8_t signature[DB_UPGATE_SIGNATURE_LENGTH];  ///< Signature of the bitstream hash
#endif
} db_upgate_start_notification_t;

///< Firmware update packet
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t index;                               ///< Index of the chunk
    uint32_t chunk_count;                         ///< Total number of chunks
    uint8_t  upgate_chunk[DB_UPGATE_CHUNK_SIZE];  ///< Bytes array of the firmware chunk
} db_upgate_pkt_t;

///< Message type
typedef enum {
    DB_UPGATE_MESSAGE_TYPE_START,
    DB_UPGATE_MESSAGE_TYPE_START_ACK,
    DB_UPGATE_MESSAGE_TYPE_CHUNK,
    DB_UPGATE_MESSAGE_TYPE_CHUNK_ACK,
} db_upgate_message_type_t;

//=========================== prototypes =======================================

/**
 * @brief   Initialize the upgate FPGA bitstream update
 *
 * @param[in]   config          Pointer to the upgate configuration
 */
void db_upgate_init(const db_upgate_conf_t *config);

/**
 * @brief   Start the upgate process
 */
void db_upgate_start(uint32_t chunk_count);

/**
 * @brief   Finalize the upgate process
 */
void db_upgate_finish(void);

/**
 * @brief   Write a chunk of the FPGA bistream to the external flash memory
 *
 * @param[in]   pkt             Pointer to the upgate packet
 */
void db_upgate_write_chunk(const db_upgate_pkt_t *pkt);

/**
 * @brief   Handle received upgate message
 *
 * @param[in]   message         The message to handle
 */
void db_upgate_handle_message(const uint8_t *message);

#endif
