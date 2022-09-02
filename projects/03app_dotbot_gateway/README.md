# DotBot gateway application

This application can be used as a gateway for communication between a computer
and a DotBot.

Plug an nRF52840DK on your computer and flash this application on it.

Use the button on the DK to control a DotBot running the 03app_dotbot application:
- button 1: drive the left wheel forward
- button 3: drive the left wheel backward
- button 2: drive the right wheel forward
- button 4: drive the right wheel backward

You can also use [bot-controller tool](https://github.com/DotBot/Botcontroller-python)
to communicate with the firmware from your computer and for example control the
DotBot using your keyboard.
