.. toctree::
   :maxdepth: 2
   :caption: Contents:


Getting started
===============

The following instructions will guide through running the default remote control example in your DotBot board.

Materials
---------

* 1x DotBot board
* 1x nRF52840-DK development board
* 1x 10pin IDC programming cable
* 1x micro USB able

Download the firmware onto the boards
-------------------------------------

1. Download the latest releases (the .hex files) of the DotBot-firmware and of the Gateway firmware from
`here <https://github.com/DotBots/DotBot-firmware/releases>`_.
You should end up with 2 files named *03app_dotbot.hex* and *03app_dotbot_gateway.hex*.
2. Connect the nRF52840-DK to your computer through the micro USB cable

.. image:: ../../../static/nRF-DK_connected.jpg
   :width: 300px
   :alt: nRF DK connected to a computer with a micro USB cable
   :align: center

A USB drive called **JLINK** should appear on your computer.

.. image:: ../../../static/JLINK_folder.png
   :width: 400px
   :alt: JLINK drive folder
   :align: center

3. Drag-&-Drop the Gateway firmware executable *03app_dotbot_gateway.hex* into the JLINK folder to program the development board.
4. Connect the nRF52840-DK to the DotBot through the 10 pin IDC cable. Make sure the DotBot is turned ON and has full batteries installed.

.. image:: ../../../static/dotbot_and_dk_connected.jpg
   :width: 300px
   :alt: DotBot connected to the nrF-DK through the 10pin IDC cable
   :align: center

5. Drag-&-Drop the DotBot firmware executable *03app_dotbot.hex* into the JLINK folder to program the DotBot.
6. Disconnect the DotBot from the nRF53840-DK

Controlling the DotBot
----------------------

.. image:: ../../../static/03app_dotbot.gif
    :alt: dotbot app demo
    :align: center

At this point you should be able to control the movement of the DotBot using the buttons on the nRF52840-DK, the controls are as follows:

* **Button 1**: Left wheel moves forward
* **Button 2**: Right wheel moves forward
* **Button 3**: Left wheel moves backward
* **Button 4**: Right wheel moves backward

You can combine buttons 1 & 2 to move the DotBot straight forward, buttons 3 & 4 to move the DotBot backward, etc.
