/**
 * @file
 * @ingroup drv_ota
 *
 * @brief  nRF52833-specific definition of the "ota" drv module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <nrf.h>

#include "nvmc.h"
#include "ota.h"
#include "partition.h"

#if defined(OTA_USE_CRYPTO)
#include "ed25519.h"
#include "sha256.h"
#include "public_key.h"
#endif

//=========================== defines ==========================================

typedef struct {
    const db_ota_conf_t  *config;
    db_partitions_table_t table;
    db_ota_cpu_type_t     cpu;
    uint8_t               reply_buffer[UINT8_MAX];
    uint32_t              target_partition;
    uint32_t              addr;
    uint32_t              last_index_acked;
#if defined(OTA_USE_CRYPTO)
    crypto_sha256_ctx_t sha256_ctx;
#endif
    uint8_t hash[DB_OTA_SHA256_LENGTH];
} db_ota_vars_t;

//=========================== variables ========================================

static db_ota_vars_t _ota_vars = { 0 };

//============================ public ==========================================

void db_ota_init(const db_ota_conf_t *config) {
    _ota_vars.config = config;
#if defined(NRF5340_XXAA_APPLICATION)
    _ota_vars.cpu = DB_OTA_CPU_NRF5340_APP;
#elif defined(NRF52840_XXAA)
    _ota_vars.cpu = DB_OTA_CPU_NRF52840;
#elif defined(NRF52833_XXAA)
    _ota_vars.cpu = DB_OTA_CPU_NRF52833;
#else
    _ota_vars.cpu = DB_OTA_CPU_UNKNOWN;
#endif
    db_read_partitions_table(&_ota_vars.table);

    if (_ota_vars.config->mode == DB_OTA_MODE_BOOTLOADER) {
        _ota_vars.target_partition = _ota_vars.table.active_image;
    } else {
        _ota_vars.target_partition = (_ota_vars.table.active_image + 1) % 2;
    }
}

void db_ota_start(void) {
    _ota_vars.last_index_acked = UINT32_MAX;
    _ota_vars.addr             = _ota_vars.table.partitions[_ota_vars.target_partition].address;
}

void db_ota_finish(void) {
    // Switch active image in partition table before resetting the device
#if defined(OTA_USE_CRYPTO)
    uint8_t hash_result[DB_OTA_SHA256_LENGTH] = { 0 };
    crypto_sha256(&_ota_vars.sha256_ctx, hash_result);

    if (memcmp(hash_result, _ota_vars.hash, DB_OTA_SHA256_LENGTH) != 0) {
        return;
    }
#endif

    if (_ota_vars.table.active_image != _ota_vars.target_partition) {
        _ota_vars.table.active_image = _ota_vars.target_partition;
        db_write_partitions_table(&_ota_vars.table);
    }
    NVIC_SystemReset();
}

void db_ota_write_chunk(const db_ota_pkt_t *pkt) {
    uint32_t addr = _ota_vars.addr + pkt->index * DB_OTA_CHUNK_SIZE;
    if (addr % DB_FLASH_PAGE_SIZE == 0) {
        db_nvmc_page_erase(addr / DB_FLASH_PAGE_SIZE);
    }
    db_nvmc_write((uint32_t *)addr, pkt->fw_chunk, DB_OTA_CHUNK_SIZE);
}

void db_ota_handle_message(const uint8_t *message) {
    db_ota_message_type_t message_type = (db_ota_message_type_t)message[0];
    switch (message_type) {
        case DB_OTA_MESSAGE_TYPE_INFO:
        {
            const db_ota_message_info_t message_info = {
                .cpu              = _ota_vars.cpu,
                .target_partition = _ota_vars.target_partition,
                .table            = _ota_vars.table,
            };
            _ota_vars.reply_buffer[0] = DB_OTA_MESSAGE_TYPE_INFO;
            memcpy(&_ota_vars.reply_buffer[1], &message_info, sizeof(db_ota_message_info_t));
            _ota_vars.config->reply(_ota_vars.reply_buffer, sizeof(db_ota_message_type_t) + sizeof(db_ota_message_info_t));
        } break;
        case DB_OTA_MESSAGE_TYPE_START:
        {
            const db_ota_start_notification_t *ota_start = (const db_ota_start_notification_t *)&message[1];
#if defined(OTA_USE_CRYPTO)
            const uint8_t *hash = ota_start->hash;
            if (!crypto_ed25519_verify(ota_start->signature, DB_OTA_SIGNATURE_LENGTH, (const uint8_t *)ota_start, sizeof(db_ota_start_notification_t) - DB_OTA_SIGNATURE_LENGTH, public_key)) {
                break;
            }
            memcpy(_ota_vars.hash, hash, DB_OTA_SHA256_LENGTH);
            crypto_sha256_init(&_ota_vars.sha256_ctx);
#else
            (void)ota_start;
#endif
            db_ota_start();
            // Acknowledge the update start
            _ota_vars.reply_buffer[0] = DB_OTA_MESSAGE_TYPE_START_ACK;
            _ota_vars.config->reply(_ota_vars.reply_buffer, sizeof(db_ota_message_type_t));
        } break;
        case DB_OTA_MESSAGE_TYPE_FW:
        {
            const db_ota_pkt_t *ota_pkt     = (const db_ota_pkt_t *)&message[1];
            const uint32_t      chunk_index = ota_pkt->index;
            const uint32_t      chunk_count = ota_pkt->chunk_count;

            if (_ota_vars.last_index_acked != chunk_index) {
                // Skip writing the chunk if already acked
                db_ota_write_chunk(ota_pkt);
#if defined(OTA_USE_CRYPTO)
                crypto_sha256_update(&_ota_vars.sha256_ctx, (const uint8_t *)ota_pkt->fw_chunk, DB_OTA_CHUNK_SIZE);
#endif
            }

            // Acknowledge the received chunk
            _ota_vars.reply_buffer[0] = DB_OTA_MESSAGE_TYPE_FW_ACK;
            memcpy(&_ota_vars.reply_buffer[1], &chunk_index, sizeof(uint32_t));
            _ota_vars.config->reply(_ota_vars.reply_buffer, sizeof(db_ota_message_type_t) + sizeof(uint32_t));
            _ota_vars.last_index_acked = chunk_index;

            if (chunk_index == chunk_count - 1) {
                db_ota_finish();
            }
        } break;
        default:
            break;
    }
}
