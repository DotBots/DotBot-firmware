# DotBot control application

This application allows the DotBot to be controlled remotely either from
- a joystick or nrf52 compatible board running a firmware that sends compatible
  commands _move_ or _rgbled_
- a computer running [the bot-controller tool](https://github.com/DotBot/Botcontroller-python)
and with a nRF52840-DK connected to it and used as gateway. The nRF52840-DK must run the
`03app_dotbot_gateway` firmware.

<!--
Keep this for future readdition
<p align="center">
  <img src="./../../static/03app_dotbot.gif" alt="dotbot app demo"/>
</p>
 -->
