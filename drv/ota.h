#ifndef __OTA_H
#define __OTA_H

/**
 * @file ota.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "ota" drv module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include "partition.h"

//=========================== defines ==========================================

#define DB_OTA_CHUNK_SIZE (128U)

typedef void (*db_ota_reply_t)(const uint8_t *, size_t);  ///< Transport agnostic function used to reply to the flasher script

typedef enum {
    DB_OTA_MODE_DEFAULT = 0,
    DB_OTA_MODE_BOOTLOADER,
} db_ota_mode_t;

typedef struct {
    db_ota_reply_t reply;
    db_ota_mode_t  mode;
} db_ota_conf_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t index;
    uint32_t chunk_count;
    uint8_t  fw_chunk[DB_OTA_CHUNK_SIZE];
} db_ota_pkt_t;

typedef enum {
    DB_OTA_CPU_NRF52833,
    DB_OTA_CPU_NRF52840,
    DB_OTA_CPU_NRF5340_APP,
    DB_OTA_CPU_UNKNOWN,
} db_ota_cpu_type_t;

typedef enum {
    DB_OTA_MESSAGE_TYPE_FW,
    DB_OTA_MESSAGE_TYPE_FW_ACK,
    DB_OTA_MESSAGE_TYPE_INFO,
} db_ota_message_type_t;

typedef struct __attribute__((packed)) {
    db_ota_cpu_type_t     cpu;
    uint32_t              target_partition;
    db_partitions_table_t table;
} db_ota_message_info_t;

//=========================== prototypes =======================================

/**
 * @brief   Initialize the OTA firmware update
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
