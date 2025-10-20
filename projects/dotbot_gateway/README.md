# DotBot gateway application

This application can be used as a gateway for communication between a computer
and a DotBot.

Plug an nRF52840-DK on your computer and flash this application on it.

Use the button on the DK to control a DotBot running the `dotbot` application:
- button 1: drive the left wheel forward
- button 3: drive the left wheel backward
- button 2: drive the right wheel forward
- button 4: drive the right wheel backward

You can also use [dotbot-controller tool](https://github.com/DotBots/PyDotBot)
to communicate with the firmware from your computer and for example control the
DotBot using your keyboard.

`dotbot-controller` is available on PyPI:

```
$ pip install dotbot-controller
```

Then use it in a command line interface:

```
$ dotbot-controller --help
usage: dotbot-controller [-h] [-t {joystick,keyboard,server}] [-p PORT] [-b BAUDRATE]

BotController, universal SailBot and DotBot controller

options:
  -h, --help            show this help message and exit
  -t {joystick,keyboard,server}, --type {joystick,keyboard,server}
                        Type of your controller. Defaults to "keyboard"
  -p PORT, --port PORT  Linux users: path to port in "/dev" folder ; Windows users: COM port. Defaults to "/dev/ttyACM0"
  -b BAUDRATE, --baudrate BAUDRATE
                        Serial baudrate. Defaults to 1000000
```
