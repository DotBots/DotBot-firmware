/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an example on how to use uzlib.
 *
 * @copyright Inria, 2024-present
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nrf.h>
#include "uzlib.h"
#include "compressed.h"

#define CHUNK_SIZE (128)

static uint8_t decompressed[1024] = { 0 };

static void _dump_decompressed(size_t length) {
    for (uint16_t pos = 0; pos < length; pos++) {
        printf("%c", decompressed[pos]);
        if (pos % 128 == 127) {
            puts("");
        }
    }
}

int main(void) {

    uzlib_init();

    struct uzlib_uncomp d;
    uzlib_uncompress_init(&d, NULL, 0);
    d.source         = compressed;
    d.source_limit   = compressed + sizeof(compressed);
    d.source_read_cb = NULL;
    d.dest_start = d.dest = decompressed;

    // Retrieve uncompressed data size from compressed chunk
    size_t dlen = compressed[sizeof(compressed) - 1];
    dlen        = 256 * dlen + compressed[sizeof(compressed) - 2];
    dlen        = 256 * dlen + compressed[sizeof(compressed) - 3];
    dlen        = 256 * dlen + compressed[sizeof(compressed) - 4];

    int ret = uzlib_gzip_parse_header(&d);
    if (ret != TINF_OK) {
        printf("Error parsing header: %d\n", ret);
        return -1;
    }

    printf("Decompressed message:\n");
    uint16_t remaining_data = dlen;
    while (remaining_data) {
        uint8_t chunk_len = remaining_data < CHUNK_SIZE ? remaining_data : CHUNK_SIZE;
        d.dest_limit      = d.dest + chunk_len;
        int ret           = uzlib_uncompress(&d);
        if (ret != TINF_OK) {
            break;
        }
        remaining_data -= chunk_len;
    }
    uint16_t dsize = d.dest - decompressed;
    assert(dsize == dlen);
    _dump_decompressed(dsize);
    printf("\nDecompressed length: %d (expected: %d)\n", dsize, dlen);

    while (1) {
        __WFE();
    }
}
