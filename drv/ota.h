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

//=========================== defines ==========================================

#define DB_OTA_CHUNK_SIZE (32U)

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t index;
    uint32_t chunk_count;
    uint8_t  fw_chunk[DB_OTA_CHUNK_SIZE];
} db_ota_pkt_t;

//=========================== prototypes =======================================

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

#endif
