/**
 * @file
 * @ingroup drv_log_flash
 *
 * @brief  Implementation of the "log_flash" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdio.h>
#include <stdint.h>

#include "nvmc.h"
#include "protocol.h"
#include "timer_hf.h"
#include "log_flash.h"

//=========================== defines ==========================================

#define DB_LOG_FLASH_START ((DB_FLASH_PAGE_SIZE * DB_FLASH_PAGE_NUM) / 2)
#define DB_LOG_FLASH_TIMER (1)

//=========================== variables ========================================

static uint32_t *_write_address = (uint32_t *)(DB_LOG_FLASH_START + DB_FLASH_OFFSET);

//=========================== public ===========================================

void db_log_flash_init(db_log_data_type_t type) {
    // Initialize the timer hf used to timestamp the logs
    db_timer_hf_init(DB_LOG_FLASH_TIMER);

    // Erase all pages that can be used to store the logs
    for (uint16_t page = DB_FLASH_PAGE_NUM / 2; page < DB_FLASH_PAGE_NUM; page++) {
        db_nvmc_page_erase(page);
    }

    db_log_header_t header = {
        .magic    = DB_LOG_MAGIC,
        .version  = DB_FIRMWARE_VERSION,
        .log_type = type,
    };

    db_nvmc_write(_write_address, &header, sizeof(db_log_header_t));
    _write_address += 1;
}

void db_log_flash_write(const void *data, size_t len) {
    if (((uint32_t)_write_address + (len >> 2) + 1) > (DB_FLASH_PAGE_NUM * DB_FLASH_PAGE_SIZE)) {
        puts("End of flash reached, skipping");
        return;
    }

    const uint32_t now = db_timer_hf_now(DB_LOG_FLASH_TIMER);
    db_nvmc_write(_write_address++, &now, sizeof(uint32_t));
    db_nvmc_write(_write_address, data, len);
    _write_address += (len >> 2);
}
