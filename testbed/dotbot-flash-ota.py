#!/usr/bin/env python

import os
import serial
import sys
import time

import click
from tqdm import tqdm
from dotbot.hdlc import hdlc_encode


BAUDRATE = 1000000
CHUNK_SIZE = 32

if sys.platform == "linux":
    DEFAULT_PORT = "/dev/ttyACM0"
elif sys.platform == "win32":
    DEFAULT_PORT = "COM3"
elif sys.platform == "Darwin":
    DEFAULT_PORT = "/dev/tty.usbmodem00000"
else:
    DEFAULT_PORT = None


@click.command()
@click.option(
    "--port", default=DEFAULT_PORT, help="Serial port to use to send the firmware."
)
@click.argument("image", type=click.File(mode="rb", lazy=True))
def main(port, image):
    fw = bytearray(image.read())
    pad_length = CHUNK_SIZE - (len(fw) % CHUNK_SIZE)
    fw = fw + bytearray(b"\xff") * (pad_length + 1)

    with serial.Serial(port, BAUDRATE, timeout=1) as ser:
        pos = 0
        progress = tqdm(
            total=len(fw), unit="B", unit_scale=False, colour="blue", ncols=100
        )
        progress.set_description(f"Sending firmware ({int(len(fw) / 1024)}kB)")
        while pos + CHUNK_SIZE < len(fw) + 1:
            buffer = bytearray()
            buffer += int(pos / CHUNK_SIZE).to_bytes(length=4, byteorder="little")
            buffer += int((len(fw) - 1) / CHUNK_SIZE).to_bytes(
                length=4, byteorder="little"
            )
            buffer += fw[pos : pos + CHUNK_SIZE]
            buffer = hdlc_encode(buffer)
            ser.write(buffer)
            delay = 0.1
            pos += CHUNK_SIZE
            progress.update(CHUNK_SIZE)
            time.sleep(delay)
        progress.update(1)
        progress.close()


if __name__ == "__main__":
    main()
