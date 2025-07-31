# Getting started

The following instructions will guide through running the default remote
control example in your DotBot board. You will flash the `03app_dotbot_gateway`
application on an nRF DK board (nRF52833-DK, nRF52840-DK or nRF5340-DK) that will
act as the remote control and you will flash the `03app_dotbot` application on
the DotBot (v1 or v2).

## Materials

- 1x DotBot board
- 1x nRF52833-DK or nRF52840-DK or nRF5340-DK development board
- 1x 10pin IDC programming cable
- 1x micro USB able

## Flashing Tools

All DotBots and DKs are based on Nordic Semiconductors microcontrollers, so
we recommend that you use Nordic programming tools to flash the firmwares:

- [nRF Util][nrf-util]
- [nRF Connect Programmer][nrf-connect-programmer]

In this document, we will give instructions about how to flash using the
`device` subcommand of `nrfutil` from [nRF Util][nrf-util], so make sure it's
installed on your computer before continuing.

## Download and flash the DotBot gateway application firmware

1. Download the latest release of the DotBot Gateway firmware (`.hex` file),
  depending on the type of DK you use:
  - [nRF52833-DK][dotbot-gateway-hex-nrf52833dk]
  - [nRF52840-DK][dotbot-gateway-hex-nrf52840dk]
  - nRF5340-DK: [application core][dotbot-gateway-hex-nrf5340dk-app] and
    [network core][hex-nrf5340dk-net]

2. Connect the nRF DK board to your computer through the micro USB cable
  ```{image} _static/images/nRF-DK_connected.jpg
  :alt: nRF DK connected to a computer with a micro USB cable
  :class: bg-primary
  :width: 300px
  :align: center
  ```

3. Flash the firmware(s) on the DK. Depending on your type of DK board, do:
  - for nrf52 based DK, run:
  ```
  nrfutil device program --x-family nrf52 --options reset=RESET_DEBUG --firmware <path to 03app_dotbot_gateway hex file>
  ```
  - for nrf53 based DK, flash both the application and network cores with the following 2 commands:
  ```
  nrfutil device program --x-family nrf53 --core Application --options reset=RESET_DEBUG --firmware <path to 03app_dotbot_gateway-nrf5340dk-app hex file>
  nrfutil device program --x-family nrf53 --core Network --options reset=RESET_DEBUG --firmware <path to 03app_nrf5340_net hex file>
  ```

4. After flashing the firmware LED1 blinks during one second to indicate that
  everything is working as expected.

## Download and flash the DotBot application firmware

1. Download the latest release of the DotBot firmware (`.hex` file),
  depending on the version of DotBot you use:
  - [DotBot v1][dotbot-hex-dotbot-v1]
  - DotBot v2: [application core][dotbot-hex-dotbot-v2] and also
    [network core][hex-nrf5340dk-net] for the network core

2. Connect the nRF DK board to your computer through the micro USB cable
  ```{image} _static/images/nRF-DK_connected.jpg
  :alt: nRF DK connected to a computer with a micro USB cable
  :class: bg-primary
  :width: 300px
  :align: center
  ```

3. Connect the nRF DK to the DotBot through the 10 pin IDC cable. Make sure the
  DotBot is turned ON and has full batteries installed
  ```{image} _static/images/dotbot_and_dk_connected.jpg
  :alt: DotBot connected to the nrF-DK through the 10pin IDC cable
  :class: bg-primary
  :width: 300px
  :align: center
  ```

3. Flash the firmware(s) on the DotBot. Depending on your DotBot version, do:
  - for v1.x (nrf52 based), run:
  ```
  nrfutil device program --x-family nrf52 --options reset=RESET_DEBUG --firmware <path to the hex file>
  ```
  - for v2.x (nrf53 based), flash both the application and network cores with the
  following 2 commands:
  ```
  nrfutil device program --x-family nrf53 --core Application --options reset=RESET_DEBUG --firmware <path to dotbot-v2 hex file>
  nrfutil device program --x-family nrf53 --core Network --options reset=RESET_DEBUG --firmware <path to 03app_nrf5340_net hex file>
  ```

4. Disconnect the DotBot from the nRF DK

## Play with the DotBot

```{image} _static/images/03app_dotbot.gif
:alt: dotbot app demo
:class: bg-primary
:align: center
```

At this point you should be able to control the movement of the DotBot using the
buttons on the nRF DK, the controls are as follows:

- **Button 1**: Left wheel moves forward
- **Button 2**: Right wheel moves forward
- **Button 3**: Left wheel moves backward
- **Button 4**: Right wheel moves backward

You can combine buttons 1 & 2 to move the DotBot straight forward, buttons 3 &
4 to move the DotBot backward, etc.


[nrf-util]: https://docs.nordicsemi.com/bundle/nrfutil/page/README.html
[nrf-connect-programmer]: https://docs.nordicsemi.com/bundle/nrf-connect-programmer/page/index.html

[dotbot-gateway-hex-nrf52833dk]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_dotbot_gateway-nrf52833dk.hex
[dotbot-gateway-hex-nrf52840dk]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_dotbot_gateway-nrf52840dk.hex
[dotbot-gateway-hex-nrf5340dk-app]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_dotbot_gateway-nrf5340dk-app.hex

[dotbot-hex-dotbot-v1]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_dotbot-dotbot-v1.hex
[dotbot-hex-dotbot-v2]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_dotbot-dotbot-v2.hex

[hex-nrf5340dk-net]: https://github.com/DotBots/DotBot-firmware/releases/latest/download/03app_nrf5340_net-nrf5340dk-net.hex
