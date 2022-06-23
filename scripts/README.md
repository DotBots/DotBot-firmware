## Python scripts for remote control

### Setup

We recommend to use virtualenv to install the dependencies and use the scripts:

1. Create a virtualenv (called `venv` here)
```
virtualenv venv
source ./venv/bin/activate
```
2. Install the scripts dependencies
```
pip install -r requirements.txt
```

### gateway_client.py usage

This script connects to the firmware running on the gateway (an nRF52840DK) via
the serial port. An nRF52840DK must be plugged via USB to the computer.
The firmware corresponds to the `03app_dotbot_gateway` project.

This script starts a simple webserver listening on port 8080 and listen for HTTP
POST requests on `/dotbot`. The POST requests should contains json data in the
form: `'{"cmd": "aGVsbG8K"}'`, where the `cmd` is base64 representation of a
cmd binary content.

If you only have one DK connected to the computer, just launch:
```
./gateway_client.py
```

In there are several DKs connected, you can specify a custom TTY port:

```
GW_TTY_PORT=/dev/ttyACM1 ./gateway_client.py
```

### keyboard_control.py usage

This script listens to keyboard events on the computer and translates them as
DotBot commands that are sent to the REST API of the gateway client script.

`MOVE RAW` and `RGBLED` commands are supported:
- pressing `r`, `g`, `b`, `y`, `p`, `w` or `n` keys changes the LED color
respectively to red, green, blue, yellow, purple, white and black (so off),
- holding pressed the left, right, top, down arrow keys control the motors.
`left+top`, `left+down`, `right+top` and `right+down` combinations are also
supported. Pressing `ctrl` wlong with arrow keys enables the _boost_ mode.
Pressing `ctrl+alt` along with arrow keys enable the _super boost_  mode (e.g.
motors are at full speed).

Just run the script as follows to communicate the gateway client running on the
same computer (e.g `localhost`):
```
./keyboard_control.py
```

If the gateway client is running on the remote computer, use the following:
```
DOTBOT_GATEWAY_URL=http://<gateway computer IP>:8080/dotbot ./keyboard_control.py
```
