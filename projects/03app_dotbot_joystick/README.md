# DotBot remote control application

This application allows the DotBot to be controlled remotely either from
- a joystick running a firmware that sends compatible _move_ commands
- a computer running the [keyboard_control.py](../../scripts/keyboard_control.py) and
with a nRF52840DK connected to it and used as gateway. The nRF52840DK must run the
`03app_dotbot_gateway` firmware.
