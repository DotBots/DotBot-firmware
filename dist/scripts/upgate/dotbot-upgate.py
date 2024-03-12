#!/usr/bin/env python

import gzip
import os
import logging
import time

from enum import Enum

import click
import serial
import structlog

from tqdm import tqdm
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from dotbot.hdlc import hdlc_encode, HDLCHandler, HDLCState
from dotbot.protocol import PROTOCOL_VERSION
from dotbot.serial_interface import SerialInterface, SerialInterfaceException


BAUDRATE = 1000000
CHUNK_SIZE = 128
PRIVATE_KEY_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "private_key")

class MessageType(Enum):
    """Types of bootloader message."""

    UPGATE_MESSAGE_TYPE_START = 0
    UPGATE_MESSAGE_TYPE_START_ACK = 1
    UPGATE_MESSAGE_TYPE_CHUNK = 2
    UPGATE_MESSAGE_TYPE_CHUNK_ACK = 3


class DotBotUpgate:
    """Class used to send an FPGA bitstream."""

    def __init__(self, port, baudrate, image, use_compression=False, secure=False):
        self.serial = SerialInterface(port, baudrate, self.on_byte_received)
        self.hdlc_handler = HDLCHandler()
        self.use_compression = use_compression
        self.secure = secure
        self.device_info = None
        self.device_info_received = False
        self.start_ack_received = False
        self.image = image
        self.last_acked_chunk = -1
        # Just write a single byte to fake a DotBot gateway handshake
        self.serial.write(int(PROTOCOL_VERSION).to_bytes(length=1))

    def on_byte_received(self, byte):
        self.hdlc_handler.handle_byte(byte)
        if self.hdlc_handler.state == HDLCState.READY:
            payload = self.hdlc_handler.payload
            if not payload:
                return
            if payload[0] == MessageType.UPGATE_MESSAGE_TYPE_START_ACK.value:
                self.start_ack_received = True
            elif payload[0] == MessageType.UPGATE_MESSAGE_TYPE_CHUNK_ACK.value:
                self.last_acked_chunk = int.from_bytes(payload[1:5], byteorder="little")

    def init(self):
        if self.secure is True:
            digest = hashes.Hash(hashes.SHA256())
            pos = 0
            while pos + CHUNK_SIZE <= len(self.image) + 1:
                digest.update(self.image[pos : pos + CHUNK_SIZE])
                pos += CHUNK_SIZE
            fw_hash = digest.finalize()
            private_key_bytes = open(PRIVATE_KEY_PATH, "rb").read()
            private_key = Ed25519PrivateKey.from_private_bytes(private_key_bytes)
        buffer = bytearray()
        buffer += int(MessageType.UPGATE_MESSAGE_TYPE_START.value).to_bytes(
            length=1, byteorder="little"
        )
        buffer += int((len(self.image) - 1) / CHUNK_SIZE).to_bytes(
            length=4, byteorder="little"
        )
        if self.use_compression is True:
            buffer += int(self.use_compression).to_bytes(length=1, byteorder="little")
        if self.secure is True:
            buffer += fw_hash
            signature = private_key.sign(bytes(buffer[1:]))
            buffer += signature
        print("Sending start update notification...")
        self.serial.write(hdlc_encode(buffer))
        timeout = 0  # ms
        while self.start_ack_received is False and timeout < 60000:
            timeout += 1
            time.sleep(0.01)
        return self.start_ack_received is True

    def run(self):
        data = self.image
        if self.use_compression:
            # Compress the image by blocks and pad it to the next block size
            data = gzip.compress(data)
            pad_length = CHUNK_SIZE - (len(data) % CHUNK_SIZE)
            data += bytearray(b"\xff") * (pad_length + 1)

        # Send the compressed image by chunks
        pos = 0
        progress = tqdm(
            total=len(data), unit="B", unit_scale=False, colour="green", ncols=100
        )
        progress.set_description(f"Flashing compressed firmware ({int(len(data) / 1024)}kB)")
        while pos + CHUNK_SIZE <= len(data) + 1:
            chunk_index = int(pos / CHUNK_SIZE)
            while self.last_acked_chunk != chunk_index:
                buffer = bytearray()
                buffer += int(MessageType.UPGATE_MESSAGE_TYPE_CHUNK.value).to_bytes(
                    length=1, byteorder="little"
                )
                buffer += int(chunk_index).to_bytes(length=4, byteorder="little")
                buffer += int((len(data) - 1) / CHUNK_SIZE).to_bytes(
                    length=4, byteorder="little"
                )
                buffer += data[pos : pos + CHUNK_SIZE]
                self.serial.write(hdlc_encode(buffer))
                time.sleep(0.005)
            pos += CHUNK_SIZE
            progress.update(CHUNK_SIZE)
        progress.update(1)
        progress.close()


@click.command()
@click.option(
    "-p",
    "--port",
    default="/dev/ttyACM0",
    help="Serial port to use to send the bitstream to the gateway.",
)
@click.option(
    "-s",
    "--secure",
    is_flag=True,
    help="Use cryptographic security (hash and signature).",
)
@click.option(
    "-c",
    "--use-compression",
    is_flag=True,
    help="Compress the bitstream before sending it.",
)
@click.option(
    "-y",
    "--yes",
    is_flag=True,
    help="Continue the upgate without prompt.",
)
@click.argument("bitstream", type=click.File(mode="rb", lazy=True))
def main(port, secure, use_compression, yes, bitstream):
    # Disable logging configure in PyDotBot
    structlog.configure(
        wrapper_class=structlog.make_filtering_bound_logger(logging.CRITICAL),
    )
    try:
        upgater = DotBotUpgate(
            port,
            BAUDRATE,
            bytearray(bitstream.read()),
            use_compression=use_compression
        )
    except (
        SerialInterfaceException,
        serial.serialutil.SerialException,
    ) as exc:
        print(f"Error: {exc}")
        return
    print(f"Image size: {len(upgater.image)}B")
    print("")
    if yes is False:
        click.confirm("Do you want to continue?", default=True, abort=True)
    ret = upgater.init()
    if ret is False:
        print("Error: No start acknowledment received. Aborting.")
        return
    upgater.run()
    print("Done")


if __name__ == "__main__":
    main()
