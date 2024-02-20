# OTAP

This directory contains 3 applications that be used for over-the-air
programming (OTAP) of DotBots:
- **otap/bootloader** contains a flexible bootloader application that
  can be used to boot an image written at a given address on the flash memory.
  The bootloader application must be compiled in **Release** mode to make it as
  small as possible so that it stays under 4kiB. By default, the bootloader reads
  the partition table stored at address 0x1000 and tries the boot the image
  corresponding to the active partition. On nRF DKs, when button 4 is kept pressed
  during reset, the bootloader will stay in bootloader mode from where 3 actions
  are possible:
  - pressing button 1 will set partition 0 as active and reset the device so it
    tries to boot the image in partition 0 the next time,
  - pressing button 2 will set partition 1 as active and reset the device so it
    tries to boot the image in partition 1 the next time,
  - using the [dotbot-flash.py](../dist/scripts/otap/dotbot-flash.py) (see below)
    provided in `dist/scripts/otap`, a new firmware
    image can be sent over UART to the active partition. Once completed the
    device has to be reset manually so it boots on the newly flashed image.

- **otap/partition0**
  contains a sample application that is linked so that
  it can be written at address 0x2000, after the bootloader and partition tables
  pages. This application simply blinks an LED (LED1 on nRF DKs) and also
  contains the logic to download 128B chunks of an image and to write them on
  partition 1,

- **otap/partition1**
  contains another sample application that is
  linked so that it can be written at address 0x41000 on nrf52833 or 0x81000 on
  nrf52840/nrf5340-app, after the image on partition 0. This application also
  simply blinks an LED (LED2 on nRF DKs) and contains the logic to download
  128B chunks of an image and to write them on partition 0.

With the default partition table used by the bootloader, the flash memory is
organized as follows:

```
0x00000000  +---------------------+
            |     bootloader      |
0x00001000  +---------------------+
            |   partition table   |
0x00002000  +---------------------+
            |                     |
            |                     |
            |     partition 0     |
            |                     |
            |                     |
0x00041000  +---------------------+
    or      |                     |
0x00081000  |                     |
            |     partition 1     |
            |                     |
            |                     |
            +---------------------+
```

The start address of partition 0 and partition 1 applications is defined in the
`<Target>_MemoryMap.xml` file available in each application directory and has to
match the start address of each partition defined in the bootloader
`main.c`.

The [dotbot-flash.py](../dist/scripts/otap/dotbot-flash.py) Python script is also provided:
it reads a .bin firmware file and sends to a device over UART. The device can either
be:
  - rebooted in bootloader mode (hold button 4 pressed during reset).
  Make sure to build the image with an offset on flash that corresponds to the
  current active partition in the partition table,
  - a DK board running running the
  [DotBot gateway](https://github.com/DotBots/Dotbot-firmware/tree/main/projects/03app_dotbot_gateway/)
  application, which is just forwarding the bytes received from UART over radio.
  In this case, the firmware is split in 128B chunks and reconstructed on flash
  by an application running a code like in
  **otap/partition0**/**otap/partition1**
  applications. Make sure that the firmware was built with the right offset on flash, following
  this rule: if the active image is on partition 0, the new firmware has to be
  built for partition 1 and vice versa.

Among different common Python packages, this script requires the
[pydotbot](https://pypi.org/project/pydotbot/) package to be installed on the
system.

To install all the Python dependencies (pydotbot, click and tqdm), run:

```
pip install -r dist/scripts/otap/requirements.txt
```
