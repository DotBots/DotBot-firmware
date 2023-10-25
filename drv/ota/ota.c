/**
 * @file ota.c
 * @addtogroup DRV
 *
 * @brief  nRF52833-specific definition of the "ota" drv module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <nrf.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "nvmc.h"
#include "ota.h"
#include "partition.h"

//=========================== defines ==========================================

typedef struct {
    db_partitions_table_t table;
    uint32_t              slot;
    uint32_t              addr;
} db_ota_vars_t;

//=========================== variables ========================================

static db_ota_vars_t _ota_vars = { 0 };

//============================ public ==========================================

void db_ota_start(void) {
    db_read_partitions_table(&_ota_vars.table);
    _ota_vars.slot = (_ota_vars.table.active_image + 1) % 2;
    _ota_vars.addr = _ota_vars.table.partitions[_ota_vars.slot].address;
}

void db_ota_finish(void) {
    // Switch active image in partition table before resetting the device
    // TODO: do more verifications (CRC, etc) before rebooting
    _ota_vars.table.active_image = _ota_vars.slot;
    db_write_partitions_table(&_ota_vars.table);
    NVIC_SystemReset();
}

void db_ota_write_chunk(const db_ota_pkt_t *pkt) {
    uint32_t addr = _ota_vars.addr + pkt->index * DB_OTA_CHUNK_SIZE;
    if (addr % DB_FLASH_PAGE_SIZE == 0) {
        db_nvmc_page_erase(addr / DB_FLASH_PAGE_SIZE);
    }
    db_nvmc_write((uint32_t *)addr, pkt->fw_chunk, DB_OTA_CHUNK_SIZE);
}
