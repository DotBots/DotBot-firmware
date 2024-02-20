#!/usr/bin/env python

import os
import logging
import time

from binascii import hexlify
from dataclasses import dataclass, field
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
SUPPORTED_CPUS = ["nrf52833", "nrf52840", "nrf5340-app", "unknown"]
PAGE_SIZE_MAP = {
    "nrf52833": 2048,
    "nrf52840": 4096,
    "nrf5340-app": 4096,
}
PRIVATE_KEY_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "private_key")

class MessageType(Enum):
    """Types of bootloader message."""

    OTA_MESSAGE_TYPE_START = 0
    OTA_MESSAGE_TYPE_START_ACK = 1
    OTA_MESSAGE_TYPE_FW = 2
    OTA_MESSAGE_TYPE_FW_ACK = 3
    OTA_MESSAGE_TYPE_INFO = 4


class CpuType(Enum):
    """Types of CPU."""

    OTA_CPU_NRF52833 = 0
    OTA_CPU_NRF52840 = 1
    OTA_CPU_NRF5340_APP = 2
    OTA_CPU_UNKNOWN = 3


@dataclass
class DeviceInfo:
    """Class that holds device information."""

    cpu: str = "unknown"
    active_partition: int = -1
    target_partition: int = -1
    partitions: list = field(default_factory=list)

    @staticmethod
    def from_bytes(data):
        device_info = DeviceInfo()
        device_info.cpu = SUPPORTED_CPUS[int.from_bytes(data[0:1], byteorder="little")]
        device_info.target_partition = int.from_bytes(data[1:5], byteorder="little")
        device_info.active_partition = int.from_bytes(data[13:17], byteorder="little")
        for partition in range(int.from_bytes(data[9:13], byteorder="little")):
            device_info.partitions.append(
                Partition(
                    address=int.from_bytes(
                        data[17 + partition * 8 : 21 + partition * 8],
                        byteorder="little",
                    ),
                    size=int.from_bytes(
                        data[21 + partition * 8 : 25 + partition * 8],
                        byteorder="little",
                    ),
                )
            )
        return device_info

    def __repr__(self):
        newline = "\n"
        device_info = (
            "Device info:\n"
            f"  - cpu: {self.cpu}{newline}"
            f"  - active partition: {self.active_partition}{newline}"
            f"  - target partition: {self.target_partition}{newline}"
        )
        if self.partitions:
            device_info += "  - partition table:\n"
            for index, partition in enumerate(self.partitions):
                device_info += (
                    f"    - partition {index}: "
                    f"[address: 0x{hexlify(partition.address.to_bytes(4, 'big')).decode()}, "
                    f"size: 0x{hexlify(partition.size.to_bytes(4, 'big')).decode()}]{newline}"
                )
        return device_info


@dataclass
class Partition:
    """Class that holds a partition information."""

    address: int
    size: int


class DotBotFlasher:
    """Class used to flash a firmware."""

    def __init__(self, port, baudrate, image):
        self.serial = SerialInterface(port, baudrate, self.on_byte_received)
        self.hdlc_handler = HDLCHandler()
        self.device_info = None
        self.device_info_received = False
        self.start_ack_received = False
        pad_length = CHUNK_SIZE - (len(image) % CHUNK_SIZE)
        self.image = image + bytearray(b"\xff") * (pad_length + 1)
        self.last_acked_chunk = -1
        # Just write a single byte to fake a DotBot gateway handshake
        self.serial.write(int(PROTOCOL_VERSION).to_bytes(length=1))

    def on_byte_received(self, byte):
        self.hdlc_handler.handle_byte(byte)
        if self.hdlc_handler.state == HDLCState.READY:
            payload = self.hdlc_handler.payload
            if not payload:
                return
            if payload[0] == MessageType.OTA_MESSAGE_TYPE_START_ACK.value:
                self.start_ack_received = True
            elif payload[0] == MessageType.OTA_MESSAGE_TYPE_FW_ACK.value:
                self.last_acked_chunk = int.from_bytes(payload[1:5], byteorder="little")
            elif payload[0] == MessageType.OTA_MESSAGE_TYPE_INFO.value:
                self.device_info = DeviceInfo.from_bytes(payload[1:])
                self.device_info_received = True

    def fetch_device_info(self):
        # while self.device_info_received is False:
        buffer = bytearray()
        buffer += int(MessageType.OTA_MESSAGE_TYPE_INFO.value).to_bytes(
            length=1, byteorder="little"
        )
        print("Fetching device info...")
        self.serial.write(hdlc_encode(buffer))
        timeout = 0  # ms
        while self.device_info_received is False and timeout < 100:
            timeout += 1
            time.sleep(0.01)

    def send_start_update(self, secure):
        if secure is True:
            digest = hashes.Hash(hashes.SHA256())
            pos = 0
            while pos + CHUNK_SIZE <= len(self.image) + 1:
                digest.update(self.image[pos : pos + CHUNK_SIZE])
                pos += CHUNK_SIZE
            fw_hash = digest.finalize()
            private_key_bytes = open(PRIVATE_KEY_PATH, "rb").read()
            private_key = Ed25519PrivateKey.from_private_bytes(private_key_bytes)
        attempts = 0
        while attempts < 3:
            buffer = bytearray()
            buffer += int(MessageType.OTA_MESSAGE_TYPE_START.value).to_bytes(
                length=1, byteorder="little"
            )
            buffer += int((len(self.image) - 1) / CHUNK_SIZE).to_bytes(
                length=4, byteorder="little"
            )
            if secure is True:
                buffer += fw_hash
                signature = private_key.sign(bytes(buffer[1:]))
                buffer += signature
            print("Sending start update notification...")
            self.serial.write(hdlc_encode(buffer))
            attempts += 1
            timeout = 0  # ms
            while self.start_ack_received is False and timeout < 1000:
                timeout += 1
                time.sleep(0.01)
            if self.start_ack_received is True:
                break
        return attempts < 3

    def flash(self):
        page_size = PAGE_SIZE_MAP[self.device_info.cpu]
        pos = 0
        progress = tqdm(
            total=len(self.image), unit="B", unit_scale=False, colour="green", ncols=100
        )
        progress.set_description(f"Flashing firmware ({int(len(self.image) / 1024)}kB)")
        while pos + CHUNK_SIZE <= len(self.image) + 1:
            chunk_index = int(pos / CHUNK_SIZE)
            while self.last_acked_chunk != chunk_index:
                buffer = bytearray()
                buffer += int(MessageType.OTA_MESSAGE_TYPE_FW.value).to_bytes(
                    length=1, byteorder="little"
                )
                buffer += int(chunk_index).to_bytes(length=4, byteorder="little")
                buffer += int((len(self.image) - 1) / CHUNK_SIZE).to_bytes(
                    length=4, byteorder="little"
                )
                buffer += self.image[pos : pos + CHUNK_SIZE]
                self.serial.write(hdlc_encode(buffer))
                delay = 0.1 if pos % page_size == 0 else 0.005
                time.sleep(delay)
            pos += CHUNK_SIZE
            progress.update(CHUNK_SIZE)
        progress.update(1)
        progress.close()


@click.command()
@click.option(
    "-p",
    "--port",
    default="/dev/ttyACM0",
    help="Serial port to use to send the firmware.",
)
@click.option(
    "-s",
    "--secure",
    is_flag=True,
    help="Use cryptographic security (hash and signature).",
)
@click.option(
    "-y",
    "--yes",
    is_flag=True,
    help="Continue flashing without prompt.",
)
@click.argument("image", type=click.File(mode="rb", lazy=True))
def main(port, secure, yes, image):
    # Disable logging configure in PyDotBot
    structlog.configure(
        wrapper_class=structlog.make_filtering_bound_logger(logging.CRITICAL),
    )
    try:
        flasher = DotBotFlasher(port, BAUDRATE, bytearray(image.read()))
    except (
        SerialInterfaceException,
        serial.serialutil.SerialException,
    ) as exc:
        print(f"Error: {exc}")
        return
    flasher.fetch_device_info()
    if flasher.device_info is None:
        print("Error: Failed to receive device info.")
        return
    if flasher.device_info.cpu not in SUPPORTED_CPUS:
        print(f"Error: CPU {flasher.cpu} is not supported.")
        return
    print(flasher.device_info)
    if not len(flasher.device_info.partitions):
        print("Error: No partition found.")
        return
    print(f"Image size: {len(flasher.image)}B")
    print(
        f"Target partition size: {flasher.device_info.partitions[flasher.device_info.active_partition].size}B"
    )
    print(f"CPU page size: {PAGE_SIZE_MAP[flasher.device_info.cpu]}B")
    print("")
    if flasher.device_info.partitions[flasher.device_info.active_partition].size < len(
        flasher.image
    ):
        print("Error: Target partition is too small.")
        return
    if yes is False:
        click.confirm("Do you want to continue?", default=True, abort=True)
    ret = flasher.send_start_update(secure)
    if ret is False:
        print("Error: No start acknowledment received. Aborting.")
        return
    flasher.flash()
    print("Done")


if __name__ == "__main__":
    main()
