/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an example on how to use LZ4 block decompression.
 *
 * @copyright Inria, 2024-present
 *
 */
#include <assert.h>
#include <stdio.h>

#include <nrf.h>

#include "lz4.h"

#include "compressed.h"

#define BUFFER_SIZE (1024)

static char _decompressed[BUFFER_SIZE] = { 0 };

static void _dump_decompressed(size_t length) {
    puts("Decompressed text:");
    for (uint16_t pos = 0; pos < length; pos++) {
        printf("%c", _decompressed[pos]);
        if (pos % 128 == 127) {
            puts("");
        }
    }
}

int main(void) {
    const uint32_t decompressed_len = LZ4_decompress_safe(
        (const char *)compressed, _decompressed, compressed_len, BUFFER_SIZE);
    if (decompressed_len == 0 || decompressed_len == 0xFFFFFFFF) {
        assert(0);
    }

    _dump_decompressed(decompressed_len);

    while (1) {
        __WFE();
    }
}
