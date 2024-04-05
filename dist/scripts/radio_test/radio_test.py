#!/usr/bin/env python3

import serial
import time
import matplotlib.pyplot as plt
from dotbot.hdlc import HDLCHandler
from dotbot.hdlc import HDLCState
from enum import Enum

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000


class blocker_type(Enum):
    NONE = 1
    TONE = 2
    BLE_1MHz = 3
    IEEE_802_15_15 = 4


def main():
    crc_count = 0
    crc_for_plot = 0
    miss_count = 0
    received_count = 0
    received_count_blocker = 0
    sum_rssi = 0
    sum_rssi_blocker = 0
    blocker_choice = 0
    save_payload_value = True
    get_rssi_blocker = False
    bit_to_bit_error = [0] * 800  # 800 bits = 100 bytes

    save_data = input(
        "Do you want to save the data txt and png ? type 'y' to say yes something else equal to no \n"
    )
    if save_data == "y":
        filename_txt = time.strftime("../../../../test/radio_test_%Y_%m_%d_%Hh%M.txt")
        filename_png = time.strftime("../../../../test/radio_test_%Y_%m_%d_%Hh%M.png")
        file = open(filename_txt, "w")

    while int(blocker_choice) not in [1, 2, 3, 4]:
        blocker_choice = input(
            "Which blocker are you using ? 1)NONE 2)TONE 3)BLE_1MHz 4)IEEE_802_15_15 (type the number to choose) \n"
        )
    print(blocker_type(int(blocker_choice)).name)

    if int(blocker_choice) != 1:
        print("You can now enable your blocker")
        get_rssi_blocker = True

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) as ser:
            hdlc_handler = HDLCHandler()
            while 1:
                data = ser.read()

                if data:
                    hdlc_handler.handle_byte(data)

                    if hdlc_handler.state == HDLCState.READY:
                        print("")
                        payload = hdlc_handler.payload
                        lenght_data_tx = int.from_bytes(
                            payload[-3].to_bytes(1, "little"), "little", signed=True
                        )
                        rssi = int.from_bytes(
                            payload[-2].to_bytes(1, "little"), "little", signed=True
                        )
                        crc = bool(payload[-1])

                        if (lenght_data_tx < 100) & (
                            get_rssi_blocker is True
                        ):  # Data received from blocker length < 100
                            print(f"rssi blocker: {rssi}")
                            sum_rssi_blocker += rssi
                            received_count_blocker += 1
                            if received_count_blocker == 99:
                                get_rssi_blocker = False

                        else:  # Data received
                            if (
                                get_rssi_blocker is False
                            ):  # wait to get RSSI from blocker

                                payload_number = payload[:-3]
                                received_count += 1

                                # Save the value for data analysis
                                if save_payload_value == True:
                                    save_payload_value = False
                                    payload_value_wanted = int.from_bytes(
                                        payload[:-3], "little", signed=True
                                    )
                                    print(f"SAVED:{payload_number}")
                                else:
                                    payload_value_wanted += 1

                                print(f"payload bytes (lsb to msb):{payload_number}")
                                print(
                                    f"lenght:{lenght_data_tx} rssi: {rssi}, crc: {crc}"
                                )
                                if save_data == "y":
                                    file.write(
                                        f"payload bytes (lsb to msb):{payload_number}\n"
                                    )
                                    file.write(
                                        f"lenght:{lenght_data_tx} rssi: {rssi}, crc: {crc}"
                                    )

                                # CRC check
                                if crc is False:
                                    crc_count += 1
                                    if save_payload_value == False:
                                        crc_for_plot += 1
                                        bit_to_bit_compare = bin(
                                            (
                                                int.from_bytes(
                                                    payload[:-3], "little", signed=True
                                                )
                                            )
                                            ^ (payload_value_wanted)
                                        )[
                                            2:
                                        ]  # check which bit are wrong
                                        print(
                                            f"Value wanted : {payload_value_wanted.to_bytes(100,'little')}"
                                        )
                                        print(
                                            f"Bit to bit xor compare (MSB to LSB) : {bit_to_bit_compare}"
                                        )
                                        for i in range(len(bit_to_bit_compare)):
                                            bit_to_bit_error[i] += int(
                                                bit_to_bit_compare[-i - 1]
                                            )

                                else:
                                    sum_rssi += rssi

                else:  # Data miss
                    if (
                        get_rssi_blocker is False
                    ):  # wait to get RSSI from blocker if their is one
                        print("Packet miss")
                        if save_data == "y":
                            file.write("Packet miss\n")
                        miss_count += 1
                        save_payload_value = True

    except (serial.SerialException, serial.SerialTimeoutException) as exc:
        print(exc)
        print("Be sure that you have chosen the good port")

    except KeyboardInterrupt:  # Results
        print("")
        print(
            f"Received {received_count} packets with {crc_count} having an invalid CRC and {miss_count} packets are missing"
        )
        if save_data == "y":
            file.write(
                f"Received {received_count} packets with {crc_count} having an invalid CRC and {miss_count} packets are missing"
            )

        if (sum_rssi != 0) & (received_count != 0):
            print(
                f"The mean RSSI of the valid packets is {sum_rssi/(received_count-crc_count)} dBm"
            )
            if save_data == "y":
                file.write(
                    f"The mean RSSI of the valid packets is {sum_rssi/(received_count-crc_count)} dBm"
                )

        if int(blocker_choice) != 1:
            print(
                f"The Blocker used is {blocker_type(int(blocker_choice)).name} with a RSSI of {sum_rssi_blocker/received_count_blocker} dBm"
            )
            if save_data == "y":
                file.write(
                    f"The Blocker used is {blocker_type(int(blocker_choice)).name} with a RSSI of {sum_rssi_blocker/received_count_blocker} dBm"
                )

        if (crc_for_plot > 0):
            bit_to_bit_error = [(i * 100 / crc_for_plot) for i in bit_to_bit_error]
            plt.scatter(range(800), bit_to_bit_error, c=bit_to_bit_error, cmap="tab20")
            plt.colorbar()
            plt.title(
                f"Percentage error per bit => blocker = {blocker_type(int(blocker_choice)).name} / {crc_for_plot} CRC errors used "
            )
            plt.xlabel("bit number")
            plt.ylabel("error[%]")
            if (save_data == "y") :
                plt.savefig(filename_png)
            plt.show()


if __name__ == "__main__":
    main()
