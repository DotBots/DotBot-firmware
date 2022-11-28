# DotBot: easy-to-use micro-robot for education and research purposes
 

<p align="center">
  <img src="static/03app_dotbot.gif" alt="dotbot app demo"/>
</p>

This repository contains the source code for the DotBot's firmware.

Loading this app onto the DotBot board will make the motors of the DotBot remote controllable by a nRF52840-DK with the [gateway-firmware](https://github.com/DotBots/Gateway-firmware/releases) firmware, as it can be seen in the above video.


## Getting Started

The following instructions will guide through running the default remote control example in your DotBot board.

### Materials

- 1x DotBot board
- 1x nRF52840-DK development board
- 1x 10pin IDC programming cable
- 1x micro USB able


### Download the firmware onto the boards


1. Download the latest releases (the .hex files) of the DotBot-firmware and of the Gateway firmware from
[here](https://github.com/DotBots/DotBot-firmware/releases)

You should end up with 2 files named `03app_dotbot.hex` and `03app_dotbot_gateway.hex`.

2. Connect the nRF52840-DK to your computer through the micro USB cable

<p align="center">
  <img src="static/nRF-DK_connected.jpg" width="40%" height="40%" alt="nRF DK connected to a computer with a micro USB cable"/>
</p>
  
  - A USB drive called  __JLINK__ should appear on your computer. 

<p align="center">
  <img src="static/JLINK_folder.png" width="60%" height="60%" alt="JLINK drive folder"/>
</p>

3. Drag-&-Drop the Gateway firmware executable `03app_dotbot_gateway.hex` into the JLINK folder to program the development board.

4. Connect the nRF52840-DK to the DotBot through the 10 pin IDC cable.
  - Make sure the DotBot is turned ON and has full batteries installed.

<p align="center">
  <img src="static/dotbot_and_dk_connected.jpg" width="40%" height="40%" alt="DotBot connected to the nrF-DK through the 10pin IDC cable."/>
</p>

5. Drag-&-Drop the DotBot firmware executable `03app_dotbot.hex` into the JLINK folder to program the DotBot.

6. Disconnect the DotBot from the nRF53840-DK


### Controlling the DotBot

<p align="center">
  <img src="static/03app_dotbot.gif" alt="dotbot app demo"/>
</p>


At this point you should be able to control the movement of the DotBot using the buttons on the nRF52840-DK, the controls are as follows:
- __Button 1__: Left wheel moves forward
- __Button 2__: Right wheel moves forward
- __Button 3__: Left wheel moves backward
- __Button 4__: Right wheel moves backward

You can combine buttons 1 & 2 to move the DotBot straight forward, buttons 3 & 4 to move the DotBot backward, etc.

## Accessing the source code

The source code of the remote control example of the DotBot can be found in
[projects/03app_dotbot/03app_dotbot.c](projects/03app_dotbot/03app_dotbot.c).
The source code of the gateway application can be found in
[projects/03app_dotbot/03app_dotbot_gateway.c](projects/03app_dotbot/03app_dotbot_gateway.c).

The different applications in this repository can be built using
[SEGGER embedded studio for ARM](https://www.segger.com/downloads/embedded-studio).
In SEGGER embedded studio, use the package manager
(available in menu Tools > Package manager) to install the CMSIS 5 CMSIS-CORE
and CMSIS-DSP packages.
