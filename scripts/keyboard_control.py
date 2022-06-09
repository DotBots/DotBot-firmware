#!/usr/bin/env python3

import base64
import os
import time

from enum import Enum

import requests
from pynput import keyboard


DOTBOT_GATEWAY_URL = os.getenv("DOTBOT_GATEWAY_URL", "http://localhost:8080/dotbot")

ACTIVE_KEYS = []
DIR_KEYS = [
    keyboard.Key.up,
    keyboard.Key.down,
    keyboard.Key.left,
    keyboard.Key.right,
]
COLOR_KEYS = ["r", "g", "b", "y", "p", "w", "n"]


class MotorSpeeds(Enum):
    NORMAL = 84
    BOOST = 100
    SUPERBOOST = 127


class Command(Enum):
    MOVE_RAW = 0
    RGB_LED = 1


def on_press(key):
    if key in ACTIVE_KEYS:
        return
    if hasattr(key, "char") and key.char in COLOR_KEYS:
        r, g, b = rgb_from_key(key.char)
        payload = bytearray()
        payload += (0).to_bytes(1, 'little')
        payload += int(Command.RGB_LED.value).to_bytes(1, 'little')
        payload += int(r).to_bytes(1, 'little')
        payload += int(g).to_bytes(1, 'little')
        payload += int(b).to_bytes(1, 'little')
        send_payload(payload)
        return
    ACTIVE_KEYS.append(key)


def on_release(key):
    if key not in ACTIVE_KEYS:
        return
    ACTIVE_KEYS.remove(key)


def speeds_from_keys():
    if any(key in ACTIVE_KEYS for key in DIR_KEYS):
        speed = MotorSpeeds.NORMAL
        if keyboard.Key.ctrl in ACTIVE_KEYS:
            speed = MotorSpeeds.BOOST
            if keyboard.Key.alt in ACTIVE_KEYS:
                speed = MotorSpeeds.SUPERBOOST
        if keyboard.Key.up in ACTIVE_KEYS and keyboard.Key.left in ACTIVE_KEYS:
            return speed.value * 0.75, speed.value
        elif keyboard.Key.up in ACTIVE_KEYS and keyboard.Key.right in ACTIVE_KEYS:
            return speed.value, speed.value * 0.75
        elif keyboard.Key.down in ACTIVE_KEYS and keyboard.Key.left in ACTIVE_KEYS:
            return -speed.value * 0.75, -speed.value
        elif keyboard.Key.down in ACTIVE_KEYS and keyboard.Key.right in ACTIVE_KEYS:
            return -speed.value, -speed.value * 0.75
        elif keyboard.Key.up in ACTIVE_KEYS:
            return speed.value, speed.value
        elif keyboard.Key.down in ACTIVE_KEYS:
            return -speed.value, -speed.value
        elif keyboard.Key.left in ACTIVE_KEYS:
            return 0, speed.value
        elif keyboard.Key.right in ACTIVE_KEYS:
            return speed.value, 0
    return 0, 0


def rgb_from_key(key):
    if key == "r":
        return 255, 0, 0
    elif key == "g":
        return 0, 255, 0
    elif key == "b":
        return 0, 0, 255
    elif key == "y":
        return 255, 255, 0
    elif key == "p":
        return 255, 0, 255
    elif key == "w":
        return 255, 255, 255
    else:  # n
        return 0, 0, 0


def send_payload(payload):
    payload_encoded = base64.b64encode(payload).decode()
    command = {"cmd": payload_encoded}
    requests.post(DOTBOT_GATEWAY_URL, json=command)


def main():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    while 1:
        left_speed, right_speed = speeds_from_keys()
        payload = bytearray()
        payload += (0).to_bytes(1, 'little')
        payload += int(Command.MOVE_RAW.value).to_bytes(1, 'little')
        payload += (0).to_bytes(1, 'little', signed=True)
        payload += int(left_speed).to_bytes(1, 'little', signed=True)
        payload += (0).to_bytes(1, 'little', signed=True)
        payload += int(right_speed).to_bytes(1, 'little', signed=True)
        send_payload(payload)
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
