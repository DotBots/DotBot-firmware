/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an example on how to use LZ4 compression/decompression.
 *
 * @copyright Inria, 2024-present
 *
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <nrf.h>

#include "lz4.h"

#define BUFFER_SIZE 2048

static char compressed_data[BUFFER_SIZE]     = { 0 };
static char decompressed_buffer[BUFFER_SIZE] = { 0 };

static const char *const src =
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit,\n"
    "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. \n"
    "Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris \n"
    "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in \n"
    "reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla \n"
    "pariatur. Excepteur sint occaecat cupidatat non proident, sunt in \n"
    "culpa qui officia deserunt mollit anim id est laborum.\n"
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit,\n"
    "sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. \n"
    "Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris \n"
    "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in \n"
    "reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla \n"
    "pariatur. Excepteur sint occaecat cupidatat non proident, sunt in \n"
    "culpa qui officia deserunt mollit anim id est laborum.\n";

int main(void) {

    /* Compression */
    const uint32_t input_length = strlen(src) + 1;
    printf("Input test length: %dB\n", input_length);

    const uint32_t compressed_data_length = LZ4_compress_default(src, compressed_data, input_length, BUFFER_SIZE);
    assert(compressed_data_length > 0);
    uint32_t compression_ratio = (compressed_data_length * 10000) / input_length;
    printf(
        "Compressed size: %dB (ratio: %u.%u)\n",
        compressed_data_length, compression_ratio / 10000, (compression_ratio / 100) % 100);

    /* Decompression */
    const uint32_t decompressed_length = LZ4_decompress_safe(
        compressed_data, decompressed_buffer, compressed_data_length, input_length);
    assert(decompressed_length > 0);
    assert(decompressed_length == input_length);
    assert(memcmp(src, decompressed_buffer, input_length) == 0);
#ifdef NDEBUG
    (void)decompressed_length;
#endif

    printf("Decompressed text:\n%s\n", decompressed_buffer);

    while (1) {
        __WFE();
    }
}
