#!/usr/bin/env python

import os
import serial
import time

import click
from tqdm import tqdm


BAUDRATE = 115200
CHUNK_SIZE = 64
SUPPORTED_CPUS = ["nrf52840", "nrf52833", "nrf5340-app", "nrf5340-net"]


@click.command()
@click.option(
    "--port", default="/dev/ttyACM0", help="Serial port to use to send the firmware."
)
@click.option(
    "--cpu",
    type=click.Choice(SUPPORTED_CPUS, case_sensitive=False),
    default="nrf52840",
    help="CPU model to flash.",
)
@click.argument("image", type=click.File(mode="rb", lazy=True))
def main(port, cpu, image):
    fw = bytearray(image.read())
    pad_length = CHUNK_SIZE - (len(fw) % CHUNK_SIZE)
    fw = fw + bytearray(b"\xff") * (pad_length + 1)

    page_size = 2048 if cpu in ["nrf52833", "nrf5340-net"] else 4096
    with serial.Serial(port, BAUDRATE, timeout=1) as ser:
        pos = 0
        progress = tqdm(total=len(fw), unit="B", unit_scale=False, colour="green")
        progress.set_description(f"Flashing firmware ({int(len(fw) / 1024)}kB)")
        while pos + CHUNK_SIZE <= len(fw):
            ser.write(fw[pos : pos + CHUNK_SIZE])
            delay = 0.2 if pos % page_size == 0 else 0.005
            pos += CHUNK_SIZE
            progress.update(CHUNK_SIZE)
            time.sleep(delay)
        progress.update(1)
        progress.close()

    # Reset device, requires nrfjprog available in PATH
    cmd = f"nrfjprog --debugreset --family {'NRF53' if 'nrf5340' in cpu else 'NRF52'}"
    if cpu == "nrf5340-net":
        cmd += " --coprocessor CP_NETWORK"
    os.system(cmd)


if __name__ == "__main__":
    main()
