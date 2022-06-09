#!/usr/bin/env python3

import base64
import serial
from bottle import post, request, run

# Examples
# curl -X POST -s -d '{"cmd": "aGVsbG8K"}' -H "Content-Type: application/json" http://localhost:8080/dotbot
# curl -X POST -s -d '{"cmd": "'$(base64 <<< azazazazazazaz)'"}' -H "Content-Type: application/json" http://localhost:8080/dotbot


@post('/dotbot')
def dotbot():
    message = base64.b64decode(request.json["cmd"])
    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        ser.write(len(message).to_bytes(1, 'little'))
        ser.write(message)


if __name__ == "__main__":
    run(host='0.0.0.0', port=8080)
