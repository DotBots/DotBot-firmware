#!/usr/bin/env python
import serial
from dotbot.hdlc import hdlc_decode
from dotbot.hdlc import HDLCHandler
from dotbot.hdlc import HDLCState

SERIAL_PORT = "COM9"
BAUDRATE = 1000000


def main():
    crc_count = 0
    miss_count = 0
    received_count = 0
    sum_rssi = 0

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01) as ser:
            hdlc_handler = HDLCHandler()
            while 1:
                data = ser.read()

                if data:
                    hdlc_handler.handle_byte(data)
                    if hdlc_handler.state == HDLCState.READY:
                        # Data received
                        payload = hdlc_handler.payload
                        payload_number = int.from_bytes(payload[:-2], "little")
                        rssi = int.from_bytes(payload[-2].to_bytes(), signed=True)
                        crc = bool(payload[-1])
                        received_count += 1
                        print(
                            f"payload number:{payload_number} ,rssi: {rssi}, crc: {crc}"
                        )

                        # CRC check
                        if crc is False:
                            crc_count += 1
                        else:
                            sum_rssi += rssi
                else:
                    # Data miss
                    print("Packet miss")
                    miss_count += 1

    except (serial.SerialException, serial.SerialTimeoutException) as exc:
        print(exc)
        print("Be sure that you have chosen the good port")

    except KeyboardInterrupt:
        # Results
        print(
            f"Received {received_count} packets with {crc_count} having an invalid CRC and {miss_count} packets are missing"
        )
        print(
            f"The mean RSSI of the valid packets is {sum_rssi/(received_count-crc_count)} dBm"
        )


if __name__ == "__main__":
    main()
