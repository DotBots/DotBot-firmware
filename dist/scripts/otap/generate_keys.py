#!/usr/bin/env python

import os

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    Encoding,
    PrivateFormat,
    PublicFormat,
    NoEncryption,
)

PRIVATE_KEY_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "private_key")
PUBLIC_KEY_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../drv/ota/public_key.h")

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
    key_pair = Ed25519PrivateKey.generate()
    private_key = key_pair.private_bytes(Encoding.Raw, PrivateFormat.Raw, NoEncryption())
    public_key = key_pair.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)

    print(f"Generated keys:\n  - private key\t: {private_key.hex()}\n  - public key\t: {public_key.hex()}")
    print(f"Saving 'private key' to {os.path.abspath(PRIVATE_KEY_PATH)}")
    with open(PRIVATE_KEY_PATH, "wb") as f:
        f.write(private_key)
    save_as_c_array(PUBLIC_KEY_PATH, "public_key", public_key.hex())

if __name__ == "__main__":
    main()
