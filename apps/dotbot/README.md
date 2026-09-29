# DotBot application

The bare DotBot app: it drives the motors and the RGB LED from commands it
receives over the radio, and nothing else. It is the smallest app that shows
the layers of the firmware, from the board drivers (`bsp/`) to the libraries
built on them (`drv/`), which makes it the one to read first.

It talks to a gateway running the `dotbot_gateway` firmware, which a computer
running [PyDotBot](https://github.com/DotBots/PyDotBot)'s controller drives. On
a DotBot v3 the radio sits on the network core, so that core runs
`nrf5340_net`.

<div align="center">

![DotBot demo](../../doc/sphinx/_static/images/03app_dotbot.gif)

</div>

## Commands

| Command | What the robot does |
|---|---|
| `CMD_WHEEL_VELOCITY` | Holds a speed per wheel, in mm/s, clamped to ±700 |
| `CMD_MOVE_RAW` | Writes a motor duty per wheel, from the joystick axes `left_y` and `right_y` (±127 to ±100 %) |
| `CMD_RGB_LED` | Sets the LED colour |
| `CONTROL_MODE` | Stops |

The console joystick, the `dotbot` keyboard and joystick tools and the buttons
of the gateway DK send `CMD_MOVE_RAW`; the controller's REST API also sends
`CMD_WHEEL_VELOCITY`.
Waypoints, the max speed and LH2 calibration are for the sandbox DotBot app
(`apps-sandbox/dotbot`), and this one ignores them.

Driving stops about 520 ms after the last `CMD_WHEEL_VELOCITY`, `CMD_MOVE_RAW`
or `CONTROL_MODE`, so a host that goes quiet cannot leave the robot running. A
host holds a speed by resending it.

## How it works

A 10 ms timer tick paces everything. On each tick the main loop:

1. applies the latest command the radio interrupt stored;
2. reads the wheel encoders (`bsp/qdec`);
3. steps the speed loop (`drv/wheel_control`), one PI controller per wheel that
   turns a setpoint in mm/s and the encoder counts into a motor duty, and
   writes it to the motors (`drv/motors`);
4. every 500 ms, sends an advertisement.

A zero speed brakes a wheel that is still turning, then lets it coast. A raw
command takes the motors from the speed loop until the next speed command or
stop.

The advertisement is the standard DotBot one, so PyDotBot lists the robot as
usual. It carries the battery, the last duty written and the encoder counts
since the previous advertisement. The app has no localization, so the position
and heading carry their unknown values and the robot does not appear on the map.
The LH2 calibration bitmask is `0xff`, as in the sandbox app, so the controller
does not send it calibration it has no use for.

## Build

```
SEGGER_DIR="<install root>" BUILD_TARGET=dotbot-v3 BUILD_CONFIG=Release make dotbot
```

The speed loop's gains are the DotBot v3 ones. The app also builds for the
other DotBot boards and the DKs, where it runs with the same gains.
