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
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nrf.h>

#include "gpio.h"
#include "n25q128.h"
#include "lz4.h"
#include "uzlib.h"
#include "upgate.h"

#if defined(UPGATE_USE_CRYPTO)
#include "ed25519.h"
#include "sha256.h"
#include "public_key.h"
#endif

//=========================== defines ==========================================

#define LZ4F_VERSION        100
#define BUFFER_SIZE         (8192)
#define UPGATE_BASE_ADDRESS (0x00000000)
#define UPGATE_TEMP_ADDRESS (0x00400000)

typedef struct {
    const db_upgate_conf_t *config;
    uint8_t                 reply_buffer[UINT8_MAX];
    uint32_t                target_partition;
    uint32_t                base_addr;
    uint32_t                addr;
    uint32_t                last_packet_acked;
    uint32_t                bistream_size;
    crypto_sha256_ctx_t     sha256_ctx;
    uint8_t                 hash[DB_UPGATE_SHA256_LENGTH];
    uint8_t                 read_buf[DB_UPGATE_CHUNK_SIZE * 2];
    uint8_t                 write_buf[DB_UPGATE_CHUNK_SIZE * 2];
    uint8_t                 compression;
    uint8_t                 temp_buffer[BUFFER_SIZE];
    uint16_t                compressed_length;
    uint8_t                 decompressed_buffer[BUFFER_SIZE];
    struct uzlib_uncomp     d;
} db_upgate_vars_t;

//=========================== variables ========================================

static db_upgate_vars_t _upgate_vars = { 0 };

//============================ public ==========================================

void db_upgate_init(const db_upgate_conf_t *config) {
    _upgate_vars.config = config;
    db_gpio_init(config->prog, DB_GPIO_OUT);
    nrf_port[config->prog->port]->PIN_CNF[config->prog->pin] |= GPIO_PIN_CNF_DRIVE_H0D1 << GPIO_PIN_CNF_DRIVE_Pos;
    db_gpio_set(_upgate_vars.config->prog);
}

void db_upgate_start(void) {
    n25q128_init(_upgate_vars.config->n25q128_conf);
    _upgate_vars.addr = UPGATE_TEMP_ADDRESS;
    // Erase the corresponding sectors.
    uint32_t sector_count = (_upgate_vars.bistream_size / N25Q128_SECTOR_SIZE) + (_upgate_vars.bistream_size % N25Q128_SECTOR_SIZE != 0);
    printf("Sectors to erase: %u\n", sector_count);
    for (uint32_t sector = 0; sector < sector_count; sector++) {
        uint32_t addr = _upgate_vars.addr + sector * N25Q128_SECTOR_SIZE;
        printf("Erasing sector %u at %p\n", sector, (uint32_t *)addr);
        n25q128_sector_erase(addr);
    }
    puts("");
    uzlib_init();
    _upgate_vars.last_packet_acked = UINT32_MAX;
    printf("Starting upgate at %p\n\n", (uint32_t *)_upgate_vars.addr);
}

void db_upgate_finish(void) {
#if defined(UPGATE_USE_CRYPTO)
    uint8_t hash_result[DB_UPGATE_SHA256_LENGTH] = { 0 };
    crypto_sha256(&_upgate_vars.sha256_ctx, hash_result);

    if (memcmp(hash_result, _upgate_vars.hash, DB_UPGATE_SHA256_LENGTH) != 0) {
        puts("Bitstream hashes don't match!");
        return;
    }
    puts("Bitstream hashes match!");
#endif

    // Move bitstream to the base location
    // Erase the corresponding sectors.
    _upgate_vars.base_addr = UPGATE_BASE_ADDRESS;
    uint32_t sector_count  = (_upgate_vars.bistream_size / N25Q128_SECTOR_SIZE) + (_upgate_vars.bistream_size % N25Q128_SECTOR_SIZE != 0);

    for (uint32_t sector = 0; sector < sector_count; sector++) {
        uint32_t addr = _upgate_vars.base_addr + sector * N25Q128_SECTOR_SIZE;
        printf("Erasing sector %u at %p\n", sector, (uint32_t *)addr);
        n25q128_sector_erase(addr);
    }
    // Copy and verify the content.
    for (uint32_t block = 0; block < _upgate_vars.bistream_size / N25Q128_PAGE_SIZE; block++) {
        printf("Moving %d bytes from %p to %p\n", N25Q128_PAGE_SIZE, (uint32_t *)(_upgate_vars.addr + block * N25Q128_PAGE_SIZE), (uint32_t *)(_upgate_vars.base_addr + block * N25Q128_PAGE_SIZE));
        n25q128_read(_upgate_vars.addr + block * N25Q128_PAGE_SIZE, _upgate_vars.temp_buffer, N25Q128_PAGE_SIZE);
        n25q128_program_page(_upgate_vars.base_addr + block * N25Q128_PAGE_SIZE, _upgate_vars.temp_buffer, N25Q128_PAGE_SIZE);
        n25q128_read(_upgate_vars.base_addr + block * N25Q128_PAGE_SIZE, _upgate_vars.read_buf, N25Q128_PAGE_SIZE);
        if (memcmp(&_upgate_vars.temp_buffer, _upgate_vars.read_buf, N25Q128_PAGE_SIZE) != 0) {
            puts("packet doesn't match!!");
        }
    }

    puts("Finishing upgate");
    // Put SPIM GPIOS as input otherwise the FPGA ends up in a broken state
    db_gpio_init(_upgate_vars.config->n25q128_conf->mosi, DB_GPIO_IN);
    db_gpio_init(_upgate_vars.config->n25q128_conf->sck, DB_GPIO_IN);
    db_gpio_init(_upgate_vars.config->n25q128_conf->miso, DB_GPIO_IN);

    // Reset the FPGA by triggerring the FPGA prog pin
    db_gpio_clear(_upgate_vars.config->prog);
    db_gpio_set(_upgate_vars.config->prog);
}

void db_upgate_handle_packet(const db_upgate_pkt_t *pkt) {
    if (_upgate_vars.compression != DB_UPGATE_COMPRESSION_NONE) {
        uint8_t packet_size = pkt->packet_size;
        _upgate_vars.compressed_length += packet_size;
        if (pkt->packet_index == 0) {
            memset(_upgate_vars.temp_buffer, 0, sizeof(_upgate_vars.temp_buffer));
            memset(_upgate_vars.decompressed_buffer, 0xff, sizeof(_upgate_vars.decompressed_buffer));
            if (_upgate_vars.compression == DB_UPGATE_COMPRESSION_GZIP) {
                uzlib_uncompress_init(&_upgate_vars.d, NULL, 0);
                _upgate_vars.d.source         = _upgate_vars.temp_buffer;
                _upgate_vars.d.source_limit   = _upgate_vars.temp_buffer + sizeof(_upgate_vars.temp_buffer);
                _upgate_vars.d.source_read_cb = NULL;
                _upgate_vars.d.dest_start = _upgate_vars.d.dest = _upgate_vars.decompressed_buffer;
            }
        }
        memcpy(&_upgate_vars.temp_buffer[pkt->packet_index * DB_UPGATE_CHUNK_SIZE], pkt->data, packet_size);
        if (pkt->packet_index == pkt->packet_count - 1) {
            if (_upgate_vars.compression == DB_UPGATE_COMPRESSION_GZIP) {
                _upgate_vars.d.source_limit = _upgate_vars.temp_buffer + _upgate_vars.compressed_length;
                int ret                     = uzlib_gzip_parse_header(&_upgate_vars.d);
                (void)ret;
                assert(ret == TINF_OK);
                uint16_t remaining_data = BUFFER_SIZE;
                while (remaining_data) {
                    uint8_t block_len         = remaining_data < DB_UPGATE_CHUNK_SIZE ? remaining_data : DB_UPGATE_CHUNK_SIZE;
                    _upgate_vars.d.dest_limit = _upgate_vars.d.dest + block_len;
                    int ret                   = uzlib_uncompress(&_upgate_vars.d);
                    if (ret != TINF_OK) {
                        break;
                    }
                    remaining_data -= block_len;
                }
            } else if (_upgate_vars.compression == DB_UPGATE_COMPRESSION_LZ4) {
                const uint32_t decompressed_len = LZ4_decompress_safe_partial((const char *)_upgate_vars.temp_buffer, (char *)_upgate_vars.decompressed_buffer, _upgate_vars.compressed_length, pkt->original_size, BUFFER_SIZE);
                (void)decompressed_len;
                assert(decompressed_len == pkt->original_size);
            } else {  // Invalid compression
                assert(0);
            }
            _upgate_vars.compressed_length = 0;
            uint32_t base_addr             = _upgate_vars.addr + pkt->chunk_index * BUFFER_SIZE;
            for (uint32_t block = 0; block < pkt->original_size / N25Q128_PAGE_SIZE; block++) {
                printf("Programming %d bytes at %p\n", N25Q128_PAGE_SIZE, (uint32_t *)(base_addr + block * N25Q128_PAGE_SIZE));
                n25q128_program_page(base_addr + block * N25Q128_PAGE_SIZE, &_upgate_vars.decompressed_buffer[block * N25Q128_PAGE_SIZE], N25Q128_PAGE_SIZE);
                n25q128_read(base_addr + block * N25Q128_PAGE_SIZE, &_upgate_vars.temp_buffer[block * N25Q128_PAGE_SIZE], N25Q128_PAGE_SIZE);
                if (memcmp(&_upgate_vars.temp_buffer[block * N25Q128_PAGE_SIZE], &_upgate_vars.temp_buffer[block * N25Q128_PAGE_SIZE], N25Q128_PAGE_SIZE) != 0) {
                    puts("packet doesn't match!!");
                }
            }
#if defined(UPGATE_USE_CRYPTO)
            crypto_sha256_update(&_upgate_vars.sha256_ctx, (const uint8_t *)_upgate_vars.decompressed_buffer, pkt->original_size);
#endif
        }
    } else {
        memcpy(&_upgate_vars.write_buf[(pkt->chunk_index % 2) * DB_UPGATE_CHUNK_SIZE], pkt->data, pkt->original_size);
        uint32_t chunk_count = (_upgate_vars.bistream_size / DB_UPGATE_CHUNK_SIZE) + ((_upgate_vars.bistream_size % DB_UPGATE_CHUNK_SIZE) != 0);
        if (pkt->chunk_index % 2 == 0 && pkt->chunk_index != chunk_count - 1) {
            return;
        }
        uint32_t addr      = _upgate_vars.addr + (pkt->chunk_index - 1) * DB_UPGATE_CHUNK_SIZE;
        size_t   data_size = (pkt->chunk_index == chunk_count - 1 && chunk_count % 2 == 1) ? pkt->original_size : DB_UPGATE_CHUNK_SIZE + pkt->original_size;
        printf("Programming %d bytes at %p\n", data_size, (uint32_t *)addr);
        n25q128_program_page(addr, _upgate_vars.write_buf, data_size);
        n25q128_read(addr, _upgate_vars.read_buf, data_size);
        if (memcmp(_upgate_vars.write_buf, _upgate_vars.read_buf, data_size) != 0) {
            puts("packet doesn't match!!");
        }
#if defined(UPGATE_USE_CRYPTO)
        crypto_sha256_update(&_upgate_vars.sha256_ctx, (const uint8_t *)_upgate_vars.read_buf, data_size);
#endif
    }
}

void db_upgate_handle_message(const uint8_t *message) {
    db_upgate_message_type_t message_type = (db_upgate_message_type_t)message[0];
    switch (message_type) {
        case DB_UPGATE_MESSAGE_TYPE_START:
        {
            const db_upgate_start_notification_t *upgate_start = (const db_upgate_start_notification_t *)&message[1];
#if defined(UPGATE_USE_CRYPTO)
            printf("Verifying ed25519 signature: ");
            const uint8_t *hash = upgate_start->hash;
            if (!crypto_ed25519_verify(upgate_start->signature, DB_UPGATE_SIGNATURE_LENGTH, (const uint8_t *)upgate_start, sizeof(db_upgate_start_notification_t) - DB_UPGATE_SIGNATURE_LENGTH, public_key)) {
                printf("Failed!\n");
                break;
            }
            printf("Success!\n");
            memcpy(_upgate_vars.hash, hash, DB_UPGATE_SHA256_LENGTH);
            crypto_sha256_init(&_upgate_vars.sha256_ctx);
#endif
            uint8_t  compression       = upgate_start->compression;
            uint32_t bitstream_size    = upgate_start->bitstream_size;
            _upgate_vars.compression   = compression;
            _upgate_vars.bistream_size = bitstream_size;
            db_upgate_start();
            // Acknowledge the update start
            _upgate_vars.reply_buffer[0] = DB_UPGATE_MESSAGE_TYPE_START_ACK;
            _upgate_vars.config->reply(_upgate_vars.reply_buffer, sizeof(db_upgate_message_type_t));
        } break;
        case DB_UPGATE_MESSAGE_TYPE_PACKET:
        {
            const db_upgate_pkt_t *upgate_pkt = (const db_upgate_pkt_t *)&message[1];
            const uint32_t         token      = upgate_pkt->packet_token;
            if (_upgate_vars.last_packet_acked != token) {
                //  Skip writing the chunk if already acked
                db_upgate_handle_packet(upgate_pkt);
            }

            // Acknowledge the received packet
            _upgate_vars.reply_buffer[0] = DB_UPGATE_MESSAGE_TYPE_PACKET_ACK;
            memcpy(&_upgate_vars.reply_buffer[1], &token, sizeof(uint32_t));
            _upgate_vars.config->reply(_upgate_vars.reply_buffer, sizeof(db_upgate_message_type_t) + sizeof(uint32_t));
            _upgate_vars.last_packet_acked = token;
        } break;
        case DB_UPGATE_MESSAGE_TYPE_FINALIZE:
            puts("");
            db_upgate_finish();
            // Acknowledge the finalize step
            _upgate_vars.reply_buffer[0] = DB_UPGATE_MESSAGE_TYPE_FINALIZE_ACK;
            _upgate_vars.config->reply(_upgate_vars.reply_buffer, sizeof(db_upgate_message_type_t));
            break;
        default:
            break;
    }
}
