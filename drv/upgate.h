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
#include "gpio.h"
#include "n25q128.h"

//=========================== defines ==========================================

#define DB_UPGATE_CHUNK_SIZE       (128U)  ///< Size of an FPGA bitstream chunk
#define DB_UPGATE_SHA256_LENGTH    (32U)
#define DB_UPGATE_SIGNATURE_LENGTH (64U)

typedef void (*db_upgate_reply_t)(const uint8_t *, size_t);  ///< Transport agnostic function used to reply to the flasher script

///< Compression modes
#define DB_UPGATE_COMPRESSION_NONE 0x00
#define DB_UPGATE_COMPRESSION_GZIP 0x01
#define DB_UPGATE_COMPRESSION_LZ4  0x02

///< FPGA bitstream update configuration
typedef struct {
    db_upgate_reply_t     reply;  ///< Pointer to the function used to reply to the upgate script
    const n25q128_conf_t *n25q128_conf;
    const gpio_t         *prog;
} db_upgate_conf_t;

///< FPGA bitstream update start notification packet
typedef struct __attribute__((packed)) {
    uint32_t bitstream_size;  ///< Size of the bitstream in bytes
    uint8_t  compression;     ///< Compression mode used
#if defined(UPGATE_USE_CRYPTO)
    uint8_t hash[DB_UPGATE_SHA256_LENGTH];          ///< SHA256 hash of the bitsream
    uint8_t signature[DB_UPGATE_SIGNATURE_LENGTH];  ///< Signature of the bitstream hash
#endif
} db_upgate_start_notification_t;

///< Firmware update packet
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t chunk_index;                 ///< Index of the chunk
    uint32_t packet_token;                ///< Random token of the packet
    uint16_t original_size;               ///< Original size
    uint8_t  packet_index;                ///< Index of the packet in the chunk
    uint8_t  packet_count;                ///< Number of packet composing the chunk
    uint8_t  packet_size;                 ///< Size of the packet
    uint8_t  data[DB_UPGATE_CHUNK_SIZE];  ///< Bytes array containing the chunk data
} db_upgate_pkt_t;

///< Message type
typedef enum {
    DB_UPGATE_MESSAGE_TYPE_START,
    DB_UPGATE_MESSAGE_TYPE_START_ACK,
    DB_UPGATE_MESSAGE_TYPE_PACKET,
    DB_UPGATE_MESSAGE_TYPE_PACKET_ACK,
    DB_UPGATE_MESSAGE_TYPE_FINALIZE,
    DB_UPGATE_MESSAGE_TYPE_FINALIZE_ACK,
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
void db_upgate_start(void);

/**
 * @brief   Finalize the upgate process
 */
void db_upgate_finish(void);

/**
 * @brief   Handle a received bitstream packet
 *
 * @param[in]   pkt             Pointer to the upgate packet
 */
void db_upgate_handle_packet(const db_upgate_pkt_t *pkt);

/**
 * @brief   Handle received upgate message
 *
 * @param[in]   message         The message to handle
 */
void db_upgate_handle_message(const uint8_t *message);

#endif
