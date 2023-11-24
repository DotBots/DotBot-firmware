#!/usr/bin/python

import os
from cryptography.hazmat.primitives import hashes

PROJECT_DIRECTORY = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "..", ".." ,"projects", "01crypto_sha256",
)
INPUT_MESSAGE = b"HashThis!"
SHA256_HASH_PATH = os.path.join(PROJECT_DIRECTORY, "expected_sha256.h")
HEADER_FORMAT = """/*
 * PLEASE DON'T EDIT
 *
 * This file was automatically generated
 */

#ifndef __{name_upper}_H
#define __{name_upper}_H

#include <stdint.h>

const uint8_t {name}[] = {{
    {data}
}};

#endif /* __{name_upper}_H */
"""


def save_as_c_array(path, name, data):
    data_str = ", ".join([f"0x{data[i:i+2]}" for i in range(0, len(data), 2)])
    print(f"Saving '{name}' to {os.path.abspath(path)}")
    with open(path, "w") as f:
        f.write(HEADER_FORMAT.format(name=name, name_upper=name.upper(), data=data_str))


def main():
    digest = hashes.Hash(hashes.SHA256())
    digest.update(INPUT_MESSAGE)
    save_as_c_array(SHA256_HASH_PATH, "expected_sha256", digest.finalize().hex())


if __name__ == "__main__":
    main()
