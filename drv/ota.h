#ifndef __OTA_H
#define __OTA_H

/**
 * @defgroup    drv_ota    Firmware update library
 * @ingroup     drv
 * @brief       Firmware update library either over the air (OTA) or via Bootloader
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include "partition.h"

//=========================== defines ==========================================

#define DB_OTA_CHUNK_SIZE       (128U)  ///< Size of a firmware chunk
#define DB_OTA_SHA256_LENGTH    (32U)
#define DB_OTA_SIGNATURE_LENGTH (64U)

typedef void (*db_ota_reply_t)(const uint8_t *, size_t);  ///< Transport agnostic function used to reply to the flasher script

///< Firmware update mode
typedef enum {
    DB_OTA_MODE_DEFAULT = 0,  ///< Default mode: OTA
    DB_OTA_MODE_BOOTLOADER,   ///< Bootloader mode
} db_ota_mode_t;

///< Firmware update configuration
typedef struct {
    db_ota_reply_t reply;  ///< Pointer to the function used to reply to the flasher script
    db_ota_mode_t  mode;   ///< Firmware update mode
} db_ota_conf_t;

///< Firmware update start notification packet
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t chunk_count;  ///< Number of chunks
#if defined(OTA_USE_CRYPTO)
    uint8_t hash[DB_OTA_SHA256_LENGTH];          ///< SHA256 hash of the firmware
    uint8_t signature[DB_OTA_SIGNATURE_LENGTH];  ///< Signature of the firmware hash
#endif
} db_ota_start_notification_t;

///< Firmware update packet
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t index;                        ///< Index of the chunk
    uint32_t chunk_count;                  ///< Total number of chunks
    uint8_t  fw_chunk[DB_OTA_CHUNK_SIZE];  ///< Bytes array of the firmware chunk
} db_ota_pkt_t;

///< CPU type
typedef enum {
    DB_OTA_CPU_NRF52833,
    DB_OTA_CPU_NRF52840,
    DB_OTA_CPU_NRF5340_APP,
    DB_OTA_CPU_UNKNOWN,
} db_ota_cpu_type_t;

///< Message type
typedef enum {
    DB_OTA_MESSAGE_TYPE_START,
    DB_OTA_MESSAGE_TYPE_START_ACK,
    DB_OTA_MESSAGE_TYPE_FW,
    DB_OTA_MESSAGE_TYPE_FW_ACK,
    DB_OTA_MESSAGE_TYPE_INFO,
} db_ota_message_type_t;

///< OTA message containing information on the running firmware
typedef struct __attribute__((packed)) {
    db_ota_cpu_type_t     cpu;
    uint32_t              target_partition;
    db_partitions_table_t table;
} db_ota_message_info_t;

//=========================== prototypes =======================================

/**
 * @brief   Initialize the OTA firmware update
 *
 * @param[in]   config          Pointer to the OTA configuration
 */
void db_ota_init(const db_ota_conf_t *config);

/**
 * @brief   Start the OTA process
 */
void db_ota_start(void);

/**
 * @brief   Finalize the OTA process: switch the active partition and reboot
 */
void db_ota_finish(void);

/**
 * @brief   Write a chunk of the firmware on the inactive partition
 *
 * @param[in]   pkt             Pointer the OTA packet
 */
void db_ota_write_chunk(const db_ota_pkt_t *pkt);

/**
 * @brief   Handle received OTA message
 *
 * @param[in]   message         The message to handle
 */
void db_ota_handle_message(const uint8_t *message);

#endif
