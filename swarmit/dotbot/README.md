# DotBot control application

This application allows the DotBot to be controlled remotely either from
- a joystick or nrf52 compatible board running a firmware that sends compatible
  commands _move_ or _rgbled_
- a computer running [the dotbot-controller tool](https://github.com/DotBots/PyDotBot)
and with a nRF52840-DK connected to it and used as gateway. The nRF52840-DK must run the
`03app_dotbot_gateway` firmware
- the buttons on the nRF52840-DK gatewaty itself

<div align="center">

![DotBot demo](../../doc/sphinx/_static/images/03app_dotbot.gif)

</div>
