/**
 * @file
 * @ingroup drv_upgate
 *
 * @brief  nRF52833-specific definition of the "upgate" drv module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2024-present
 */
#include <nrf.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lz4.h"
#include "n25q128.h"
#include "upgate.h"

#if defined(UPGATE_USE_CRYPTO)
#include "ed25519.h"
#include "sha256.h"
#include "public_key.h"
#endif

//=========================== defines ==========================================

#define LZ4F_VERSION 100
#define DECOMPRESS_BUFFER_SIZE  (4096)

typedef struct {
    const db_upgate_conf_t *config;
    uint8_t                 reply_buffer[UINT8_MAX];
    uint32_t                target_partition;
    uint32_t                addr;
    uint32_t                last_index_acked;
    uint8_t                 hash[DB_UPGATE_SHA256_LENGTH];
    uint8_t                 read_buf[DB_UPGATE_CHUNK_SIZE * 2];
    uint8_t                 write_buf[DB_UPGATE_CHUNK_SIZE * 2];
    uint8_t                 write_buf_pos;
    uint8_t                 decompress_buffer[DECOMPRESS_BUFFER_SIZE];
} db_upgate_vars_t;

//=========================== variables ========================================

static db_upgate_vars_t _upgate_vars = { 0 };
//static LZ4F_decompressionContext_t dctx;

//============================ public ==========================================

void db_upgate_init(const db_upgate_conf_t *config) {
    n25q128_init(config->n25q128_conf);
    _upgate_vars.config = config;
    //LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION);
}

void db_upgate_start(uint32_t chunk_count) {
    (void)chunk_count;
    n25q128_base_address(&_upgate_vars.addr);
    // Erase the corresponding sectors.
    uint32_t total_bytes  = chunk_count * DB_UPGATE_CHUNK_SIZE;
    uint32_t sector_count = (total_bytes / N25Q128_SECTOR_SIZE) + 1;
    printf("Sectors to erase: %u\n", sector_count);
    for (uint32_t sector = 0; sector < sector_count; sector++) {
        uint32_t addr = _upgate_vars.addr + sector * N25Q128_SECTOR_SIZE;
        printf("Erasing sector %u at %p\n", sector, addr);
        n25q128_sector_erase(addr);
    }
    puts("");
    _upgate_vars.last_index_acked = UINT32_MAX;
    //LZ4F_resetDecompressionContext(dctx);
    printf("Starting upgate at %p\n\n", _upgate_vars.addr);
}

void db_upgate_finish(void) {
    puts("Finishing upgate");
#if defined(UPGATE_USE_CRYPTO)
    uint8_t hash_result[DB_UPGATE_SHA256_LENGTH] = { 0 };
    crypto_sha256(hash_result);

    if (memcmp(hash_result, _upgate_vars.hash, DB_UPGATE_SHA256_LENGTH) != 0) {
        return;
    }
#endif

    // TODO: Reset the FPGA
}

void db_upgate_write_chunk(const db_upgate_pkt_t *pkt) {
    int32_t decompressed_length = LZ4_decompress_safe(
        (const char *)pkt->upgate_chunk, (char *)_upgate_vars.decompress_buffer, DB_UPGATE_CHUNK_SIZE, DECOMPRESS_BUFFER_SIZE);
    assert(decompressed_length < DECOMPRESS_BUFFER_SIZE);
    uint32_t decompress_buffer_pos = 0;
    do {
        uint16_t write_buf_available = (DB_UPGATE_CHUNK_SIZE * 2) - _upgate_vars.write_buf_pos;
        memcpy(&_upgate_vars.write_buf[_upgate_vars.write_buf_pos], &_upgate_vars.decompress_buffer[decompress_buffer_pos], write_buf_available);
        decompressed_length -= write_buf_available;
        decompress_buffer_pos += write_buf_available;
        _upgate_vars.write_buf_pos = 0;
        uint32_t addr = _upgate_vars.addr + (pkt->index - 1) * DB_UPGATE_CHUNK_SIZE;

        printf("Programming 256 bytes at %p\n", addr);
        n25q128_program_page(addr, _upgate_vars.write_buf, DB_UPGATE_CHUNK_SIZE * 2);
        n25q128_read(addr, _upgate_vars.read_buf, DB_UPGATE_CHUNK_SIZE * 2);
        if (memcmp(_upgate_vars.write_buf, _upgate_vars.read_buf, DB_UPGATE_CHUNK_SIZE * 2) != 0) {
            puts("packet doesn't match!!");
        }
    } while (decompressed_length >= (int32_t)DB_UPGATE_CHUNK_SIZE * 2);
    memcpy(&_upgate_vars.write_buf[_upgate_vars.write_buf_pos], &_upgate_vars.decompress_buffer[decompress_buffer_pos], decompressed_length);
    _upgate_vars.write_buf_pos = decompressed_length;


    //memcpy(&_upgate_vars.write_buf[(pkt->index % 2) * DB_UPGATE_CHUNK_SIZE], pkt->upgate_chunk, DB_UPGATE_CHUNK_SIZE);
    //if (pkt->index % 2 == 0) {
    //    return;
    //}
    //uint32_t addr = _upgate_vars.addr + (pkt->index - 1) * DB_UPGATE_CHUNK_SIZE;

    //printf("Programming 256 bytes at %p\n", addr);
    //n25q128_program_page(addr, _upgate_vars.write_buf, DB_UPGATE_CHUNK_SIZE * 2);
    //n25q128_read(addr, _upgate_vars.read_buf, DB_UPGATE_CHUNK_SIZE * 2);
    //if (memcmp(_upgate_vars.write_buf, _upgate_vars.read_buf, DB_UPGATE_CHUNK_SIZE * 2) != 0) {
    //    puts("packet doesn't match!!");
    //}
}

void db_upgate_handle_message(const uint8_t *message) {
    db_upgate_message_type_t message_type = (db_upgate_message_type_t)message[0];
    switch (message_type) {
        case DB_UPGATE_MESSAGE_TYPE_START:
        {
            const db_upgate_start_notification_t *upgate_start = (const db_upgate_start_notification_t *)&message[1];
#if defined(UPGATE_USE_CRYPTO)
            const uint8_t *hash = upgate_start->hash;
            if (!crypto_ed25519_verify(upgate_start->signature, DB_UPGATE_SIGNATURE_LENGTH, (const uint8_t *)upgate_start, sizeof(db_upgate_start_notification_t) - DB_UPGATE_SIGNATURE_LENGTH, public_key)) {
                break;
            }
            memcpy(_upgate_vars.hash, hash, DB_UPGATE_SHA256_LENGTH);
            crypto_sha256_init();
#endif
            db_upgate_start(upgate_start->chunk_count);
            // Acknowledge the update start
            _upgate_vars.reply_buffer[0] = DB_UPGATE_MESSAGE_TYPE_START_ACK;
            _upgate_vars.config->reply(_upgate_vars.reply_buffer, sizeof(db_upgate_message_type_t));
        } break;
        case DB_UPGATE_MESSAGE_TYPE_CHUNK:
        {
            const db_upgate_pkt_t *upgate_pkt  = (const db_upgate_pkt_t *)&message[1];
            const uint32_t         chunk_index = upgate_pkt->index;
            const uint32_t         chunk_count = upgate_pkt->chunk_count;

            if (_upgate_vars.last_index_acked != chunk_index) {
                //  Skip writing the chunk if already acked
                db_upgate_write_chunk(upgate_pkt);
#if defined(UPGATE_USE_CRYPTO)
                crypto_sha256_update((const uint8_t *)upgate_pkt->upgate_chunk, DB_UPGATE_CHUNK_SIZE);
#endif
            }

            // Acknowledge the received chunk
            _upgate_vars.reply_buffer[0] = DB_UPGATE_MESSAGE_TYPE_CHUNK_ACK;
            memcpy(&_upgate_vars.reply_buffer[1], &chunk_index, sizeof(uint32_t));
            _upgate_vars.config->reply(_upgate_vars.reply_buffer, sizeof(db_upgate_message_type_t) + sizeof(uint32_t));
            _upgate_vars.last_index_acked = chunk_index;

            if (chunk_index == chunk_count - 1) {
                puts("");
                db_upgate_finish();
            }
        } break;
        default:
            break;
    }
}
