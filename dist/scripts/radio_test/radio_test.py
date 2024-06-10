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
    tone = 2
    ble = 3
    ieee_802154 = 4


def main():
    crc_count = 0
    crc_for_plot = 0
    miss_count = 0
    received_count = 0
    received_count_blocker = 0
    sum_rssi = 0
    sum_rssi_blocker = 0
    blocker_choice = 0
    com_choice = 0
    save_payload_value = True
    get_rssi_blocker = False
    bit_to_bit_error = [0] * 800  # 100 bytes

    while int(com_choice) not in [1, 2]:
        com_choice = input(
            "Which comminication are you using ? 1)BLE 2)IEEE_802_15_4 (type the number to choose) \n"
        )
    print(blocker_type(int(com_choice) + 2).name)

    while int(blocker_choice) not in [1, 2, 3, 4]:
        blocker_choice = input(
            "Which blocker are you using ? 1)NONE 2)TONE 3)BLE 4)IEEE_802_15_4(type the number to choose) \n"
        )
    print(blocker_type(int(blocker_choice)).name)

    save_data = input(
        "Do you want to save the data txt and png ? type 'y' to say yes something else equal to no \n"
    )
    if save_data == "y":
        filename_txt = time.strftime("../../../../test/radio_test_%Y_%m_%d_%Hh%M.txt")
        filename_png = time.strftime("../../../../test/radio_test_%Y_%m_%d_%Hh%M.png")
        file = open(filename_txt, "w")

    if int(blocker_choice) != 1:
        print("Use blocker_rssi to get rssi from blocker")
        get_rssi_blocker = True

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01) as ser:
            hdlc_handler = HDLCHandler()
            while (received_count +miss_count) < 10000:
                data = ser.read()
                if data:

                    hdlc_handler.handle_byte(data)

                    if hdlc_handler.state == HDLCState.READY:
                        payload = hdlc_handler.payload
                        if payload:
                            lenght_payload_tx = int.from_bytes(
                                payload[-3].to_bytes(1, "little"), "little", signed=True
                            )
                            rssi = int.from_bytes(
                                payload[-2].to_bytes(1, "little"), "little", signed=True
                            )
                            crc = bool(payload[-1])
                            
                            # Data received from blocker
                            if (get_rssi_blocker is True) & (lenght_payload_tx == 10):
                                print(f"rssi blocker: {rssi}")
                                sum_rssi_blocker += rssi
                                received_count_blocker += 1
                                print(payload[0])
                                if received_count_blocker == 100:
                                    input("Change code to BLOCKER")
                                    get_rssi_blocker = False

                            else:  # Data received
                                # wait to get RSSI from blocker
                                if (get_rssi_blocker is False) : 
                                    if (lenght_payload_tx == 125 ) :
                                        if save_payload_value == False:
                                            payload_value_wanted += 1
                                        print("Packet miss")
                                        miss_count += 1
                                
                                    else :   
                                        if int(com_choice) == 1:
                                            payload_number = payload[:-3]
                                        else:  # avoid LQI that is on the payload
                                            payload_number = payload[:-5]
                                            lenght_payload_tx -= 2

                                        received_count += 1

                                        # Save the value for data analysis
                                        if save_payload_value == True:
                                            if int(com_choice) == 1:
                                                payload_value_wanted = int.from_bytes(
                                                    payload[:-3], "little", signed=True
                                                )
                                            else:
                                                payload_value_wanted = int.from_bytes(
                                                    payload[:-5], "little", signed=True
                                                )
                                            save_payload_value = False

                                        else:
                                            payload_value_wanted += 1

                                        print("")
                                        print(
                                            f"payload bytes (lsb to msb):{payload_number}"
                                        )
                                        print(
                                            f"lenght:{lenght_payload_tx} rssi: {rssi}, crc: {crc}"
                                        )

                                        sum_rssi += rssi

                                        # CRC check
                                        if crc is False:
                                            crc_count += 1
                                            if save_payload_value == False:
                                                crc_for_plot += 1

                                                if int(com_choice) == 1:
                                                    # check which bit are wrong
                                                    bit_to_bit_compare = bin(
                                                        abs(
                                                            (
                                                                int.from_bytes(
                                                                    payload[:-3],
                                                                    "little",
                                                                    signed=True,
                                                                )
                                                            )
                                                            ^ (payload_value_wanted)
                                                        )
                                                    )[2:]
                                                    print(
                                                        f"Value wanted : {payload_value_wanted.to_bytes(100,'little')}"
                                                    )
                                                else:
                                                    # check which bit are wrong
                                                    bit_to_bit_compare = bin(
                                                        abs(
                                                            (
                                                                int.from_bytes(
                                                                    payload[:-5],
                                                                    "little",
                                                                    signed=True,
                                                                )
                                                            )
                                                            ^ (payload_value_wanted)
                                                        )
                                                    )[2:]
                                                    print(
                                                        f"Value wanted : {payload_value_wanted.to_bytes(98,'little')}"
                                                    )

                                                print(
                                                    f"Bit to bit xor compare (MSB to LSB) : {bit_to_bit_compare}"
                                                )
                                                for i in range(len(bit_to_bit_compare)):
                                                    bit_to_bit_error[i] += int(
                                                        bit_to_bit_compare[-i - 1]
                                                    )

                else:  # Data miss
                    # wait to get RSSI from blocker if their is one
                    if get_rssi_blocker is False:
                        if save_payload_value == False:
                            payload_value_wanted += 1
                        print("Packet miss")
                        miss_count += 1

    except (serial.SerialException, serial.SerialTimeoutException) as exc:
        print(exc)
        print("Be sure that you have chosen the good port")

    except KeyboardInterrupt:  # Results
        print("")
        print(f"Keyboard Interrupt")
    
    # print results
    print("")
    print(f"Communication used {blocker_type(int(com_choice)+2).name}")
    print(
        f"Received {received_count} packets with {crc_count} having an invalid CRC and {miss_count} packets are missing"
    )
    if save_data == "y":
        file.write(f"Communication used {blocker_type(int(com_choice)+2).name}\n")
        file.write(
            f"Received {received_count} packets with {crc_count} having an invalid CRC and {miss_count} packets are missing\n"
        )

    if (sum_rssi != 0) & (received_count != 0):
        print(
            f"The mean RSSI of the valid packets is {sum_rssi/(received_count)} dBm"
        )
        if save_data == "y":
            file.write(
                f"The mean RSSI of the valid packets is {sum_rssi/(received_count)} dBm\n"
            )

    if int(blocker_choice) != 1:
        print(
            f"The Blocker used is {blocker_type(int(blocker_choice)).name} with a RSSI of {sum_rssi_blocker/received_count_blocker} dBm"
        )
        if save_data == "y":
            file.write(
                f"The Blocker used is {blocker_type(int(blocker_choice)).name} with a RSSI of {sum_rssi_blocker/received_count_blocker} dBm\n"
            )

    if crc_for_plot > 0:
        bit_to_bit_error = [(i * 100 / crc_for_plot) for i in bit_to_bit_error]
        plt.scatter(range(800), bit_to_bit_error, c=bit_to_bit_error, cmap="tab20")
        plt.colorbar()
        plt.title(
            f"Percentage error per bit => blocker = {blocker_type(int(blocker_choice)).name} / {crc_for_plot} CRC errors used "
        )
        plt.xlabel("bit number")
        plt.ylabel("error[%]")
        if save_data == "y":
                plt.savefig(filename_png)
        plt.show()


if __name__ == "__main__":
    main()
