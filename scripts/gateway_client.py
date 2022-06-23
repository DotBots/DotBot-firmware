#!/usr/bin/env python3

import base64
import os
import serial
from bottle import post, request, run

# Examples
# curl -X POST -s -d '{"cmd": "aGVsbG8K"}' -H "Content-Type: application/json" http://localhost:8080/dotbot
# curl -X POST -s -d '{"cmd": "'$(base64 <<< azazazazazazaz)'"}' -H "Content-Type: application/json" http://localhost:8080/dotbot


GW_TTY_PORT = os.getenv("GW_TTY_PORT", "/dev/ttyACM0")
GW_TTY_BAUDRATE = int(os.getenv("GW_TTY_BAUDRATE", 115200))


@post('/dotbot')
def dotbot():
    message = base64.b64decode(request.json["cmd"])
    with serial.Serial(GW_TTY_PORT, GW_TTY_BAUDRATE, timeout=1) as ser:
        ser.write(len(message).to_bytes(1, 'little'))
        ser.write(message)


if __name__ == "__main__":
    run(host='0.0.0.0', port=8080)
