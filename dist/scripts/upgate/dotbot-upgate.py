#!/usr/bin/env python

import gzip
import os
import logging
import secrets
import time

from dataclasses import dataclass
from enum import Enum

import click
import lz4.block
import serial
import structlog

from tqdm import tqdm
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from dotbot.hdlc import hdlc_encode, HDLCHandler, HDLCState
from dotbot.protocol import PROTOCOL_VERSION
from dotbot.serial_interface import SerialInterface, SerialInterfaceException


SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000
CHUNK_SIZE = 128
COMPRESSED_CHUNK_SIZE = 8192
PRIVATE_KEY_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "private_key")

class MessageType(Enum):
    """Types of bootloader message."""

    UPGATE_MESSAGE_TYPE_START = 0
    UPGATE_MESSAGE_TYPE_START_ACK = 1
    UPGATE_MESSAGE_TYPE_PACKET = 2
    UPGATE_MESSAGE_TYPE_PACKET_ACK = 3
    UPGATE_MESSAGE_TYPE_FINALIZE = 4
    UPGATE_MESSAGE_TYPE_FINALIZE_ACK = 5


class CompressionMode(Enum):
    """Types of compression."""

    UPGATE_COMPRESSION_NONE = 0
    UPGATE_COMPRESSION_GZIP = 1
    UPGATE_COMPRESSION_LZ4 = 2


COMPRESSION_MODES_MAP = {
    "none": CompressionMode.UPGATE_COMPRESSION_NONE,
    "gzip": CompressionMode.UPGATE_COMPRESSION_GZIP,
    "lz4": CompressionMode.UPGATE_COMPRESSION_LZ4,
}


@dataclass
class RadioPacket:
    """Class that holds radio packets."""

    index: int
    token: bytes
    data: bytes


@dataclass
class DataChunk:
    """Class that holds data chunks."""

    index: int
    dsize: int
    csize: int
    packets: bytes


class DotBotUpgate:
    """Class used to send an FPGA bitstream."""

    def __init__(self, port, baudrate, image, compression="none", secure=False):
        self.serial = SerialInterface(port, baudrate, self.on_byte_received)
        self.hdlc_handler = HDLCHandler()
        self.compression = compression
        self.secure = secure
        self.device_info = None
        self.device_info_received = False
        self.start_ack_received = False
        self.finalize_ack_received = False
        self.image = image
        self.last_acked_token = -1
        self.chunks = []
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
            if payload[0] == MessageType.UPGATE_MESSAGE_TYPE_FINALIZE_ACK.value:
                self.finalize_ack_received = True
            elif payload[0] == MessageType.UPGATE_MESSAGE_TYPE_PACKET_ACK.value:
                self.last_acked_token = payload[1:5].hex()

    def init(self):
        private_key_bytes = open(PRIVATE_KEY_PATH, "rb").read()
        private_key = Ed25519PrivateKey.from_private_bytes(private_key_bytes)
        digest = hashes.Hash(hashes.SHA256())
        if self.compression == "none":
            chunks_count = int(len(self.image) / CHUNK_SIZE) + int(len(self.image) % CHUNK_SIZE != 0)
            for chunk in range(chunks_count):
                if chunk == chunks_count - 1:
                    dsize = len(self.image) % CHUNK_SIZE
                else:
                    dsize = CHUNK_SIZE
                data = self.image[chunk * CHUNK_SIZE : chunk * CHUNK_SIZE + dsize]
                digest.update(data)
                self.chunks.append(
                    DataChunk(
                        index=chunk,
                        dsize=dsize,
                        csize=dsize,
                        packets=[
                            RadioPacket(index=0, token=secrets.token_bytes(4), data=data)
                        ],
                    )
                )
        else:
            chunks_count = int(len(self.image) / COMPRESSED_CHUNK_SIZE) + int(len(self.image) % COMPRESSED_CHUNK_SIZE != 0)
            for chunk in range(chunks_count):
                if chunk == chunks_count - 1:
                    dsize = len(self.image) % COMPRESSED_CHUNK_SIZE
                else:
                    dsize = COMPRESSED_CHUNK_SIZE
                data = self.image[chunk * COMPRESSED_CHUNK_SIZE : chunk * COMPRESSED_CHUNK_SIZE + dsize]
                digest.update(data)
                if self.compression == "gzip":
                    compressed = gzip.compress(data)
                elif self.compression == "lz4":
                    compressed = lz4.block.compress(data, mode="high_compression", store_size=False)
                else:
                    compressed = []
                packets_count = int(len(compressed) / CHUNK_SIZE) + int(len(compressed) % CHUNK_SIZE != 0)
                packets = []
                for packet_idx in range(packets_count):
                    if packet_idx == packets_count - 1:
                        packet_data = compressed[packet_idx * CHUNK_SIZE:]
                    else:
                        packet_data = compressed[packet_idx * CHUNK_SIZE : (packet_idx + 1) * CHUNK_SIZE]
                    packets.append(
                        RadioPacket(index=packet_idx, token=secrets.token_bytes(4), data=packet_data)
                    )
                self.chunks.append(
                    DataChunk(
                        index=chunk,
                        dsize=dsize,
                        csize=len(compressed),
                        packets=packets,
                    )
                )
            image_size = len(self.image)
            compressed_size = sum([c.csize for c in self.chunks])
            print(f"Compression ratio: {(1 - compressed_size / image_size) * 100:.2f}% ({image_size}B -> {compressed_size}B)")
            print(f"Compressed chunks ({COMPRESSED_CHUNK_SIZE}B): {len(self.chunks)}")
        print(f"Radio packets ({CHUNK_SIZE}B): {sum([len(c.packets) for c in self.chunks])}")
        fw_hash = digest.finalize()

        buffer = bytearray()
        buffer += int(MessageType.UPGATE_MESSAGE_TYPE_START.value).to_bytes(
            length=1, byteorder="little"
        )
        buffer += len(self.image).to_bytes(length=4, byteorder="little")
        buffer += int(COMPRESSION_MODES_MAP[self.compression].value).to_bytes(length=1, byteorder="little")
        if self.secure is True:
            buffer += fw_hash
            signature = private_key.sign(bytes(buffer[1:]))
            buffer += signature
        print("Sending start upgate notification...")
        self.serial.write(hdlc_encode(buffer))
        timeout = 0  # ms
        while self.start_ack_received is False and timeout < 10000:
            timeout += 1
            time.sleep(0.01)
        return self.start_ack_received is True

    def finalize(self):
        buffer = bytearray()
        buffer += int(MessageType.UPGATE_MESSAGE_TYPE_FINALIZE.value).to_bytes(
            length=1, byteorder="little"
        )
        buffer += int((len(self.image) - 1) / CHUNK_SIZE).to_bytes(
            length=4, byteorder="little"
        )
        print("Sending upgate finalize...")
        self.serial.write(hdlc_encode(buffer))
        timeout = 0  # ms
        while self.finalize_ack_received is False and timeout < 10000:
            timeout += 1
            time.sleep(0.01)
        return self.finalize_ack_received is True

    def send_packet(self, chunk, packet):
        send_time = time.time()
        send = True
        tries = 0
        while tries < 3:
            if self.last_acked_token == packet.token.hex():
                break
            if send is True:
                buffer = bytearray()
                buffer += int(MessageType.UPGATE_MESSAGE_TYPE_PACKET.value).to_bytes(
                    length=1, byteorder="little"
                )
                buffer += int(chunk.index).to_bytes(length=4, byteorder="little")
                buffer += packet.token
                buffer += int(chunk.dsize).to_bytes(length=2, byteorder="little")
                buffer += int(packet.index).to_bytes(length=1, byteorder="little")
                buffer += int(len(chunk.packets)).to_bytes(length=1, byteorder="little")
                buffer += int(len(packet.data)).to_bytes(length=1, byteorder="little")
                buffer += packet.data
                self.serial.write(hdlc_encode(buffer))
                send_time = time.time()
                tries += 1
            time.sleep(0.01)
            send = time.time() - send_time > 5
        else:
            raise Exception(f"packet #{chunk.index} ({packet.token.hex()}) not acknowledged. Aborting.")

    def transfer(self):
        if self.compression in ["gzip", "lz4"]:
            data_size = sum([c.csize for c in self.chunks])
        else:
            data_size = len(self.image)
        progress = tqdm(range(0, data_size), unit="B", unit_scale=False, colour="green", ncols=100)
        progress.set_description(f"Flashing compressed firmware ({int(data_size / 1024)}kB)")
        for chunk in self.chunks:
            for packet in chunk.packets:
                self.send_packet(chunk, packet)
            progress.update(chunk.csize)
        progress.close()

@click.command()
@click.option(
    "-p",
    "--port",
    default=SERIAL_PORT,
    help=f"Serial port to use to send the bitstream to the gateway. Default: {SERIAL_PORT}.",
)
@click.option(
    "-b",
    "--baudrate",
    default=BAUDRATE,
    help=f"Serial port baudrate. Default: {BAUDRATE}.",
)
@click.option(
    "-s",
    "--secure",
    is_flag=True,
    help="Use cryptographic security (hash and signature).",
)
@click.option(
    "-c",
    "--compression",
    type=click.Choice(['none', 'gzip', 'lz4']),
    default="none",
    help="Bitstream compression mode.",
)
@click.option(
    "-y",
    "--yes",
    is_flag=True,
    help="Continue the upgate without prompt.",
)
@click.argument("bitstream", type=click.File(mode="rb", lazy=True))
def main(port, baudrate, secure, compression, yes, bitstream):
    # Disable logging configure in PyDotBot
    structlog.configure(
        wrapper_class=structlog.make_filtering_bound_logger(logging.CRITICAL),
    )
    try:
        upgater = DotBotUpgate(
            port,
            baudrate,
            bytearray(bitstream.read()),
            compression=compression,
            secure=secure,
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
        print("Error: No start acknowledgment received. Aborting.")
        return
    try:
        upgater.transfer()
    except Exception as exc:
        print(f"Error during transfer: {exc}")
        return
    ret = upgater.finalize()
    if ret is False:
        print("Error: No finalize acknowledgment received. Upgate failed.")
        return
    print("Done")


if __name__ == "__main__":
    main()
