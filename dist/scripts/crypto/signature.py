#!/usr/bin/python

import os

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    Encoding,
    PrivateFormat,
    PublicFormat,
    NoEncryption,
)

PROJECT_DIRECTORY = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "..", ".." ,"projects", "01crypto_ed25519",
)
print(PROJECT_DIRECTORY)
PRIVATE_KEY_PATH = os.path.join(PROJECT_DIRECTORY, "private_key.h")
PUBLIC_KEY_PATH = os.path.join(PROJECT_DIRECTORY, "public_key.h")
EXPECTED_SIGNATURE_PATH = os.path.join(PROJECT_DIRECTORY, "expected_signature.h")

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
    private_key = Ed25519PrivateKey.generate()
    public_key = private_key.public_key()
    expected_signature = private_key.sign(b"SignThis!")

    save_as_c_array(
        PRIVATE_KEY_PATH,
        "private_key",
        private_key.private_bytes(
            Encoding.Raw, PrivateFormat.Raw, NoEncryption()
        ).hex(),
    )
    save_as_c_array(
        PUBLIC_KEY_PATH,
        "public_key",
        public_key.public_bytes(Encoding.Raw, PublicFormat.Raw).hex(),
    )
    save_as_c_array(
        EXPECTED_SIGNATURE_PATH, "expected_signature", expected_signature.hex()
    )


if __name__ == "__main__":
    main()
