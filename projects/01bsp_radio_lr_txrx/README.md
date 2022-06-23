# Long Range Radio TX-RX board support package usage example


Loading this app onto the DotBot board will make the DotBot able to comunicate using Long Range BLE radio. 

You can test this code by loading the FW onto 2x nRF52840 DKs and have one DK in the Debug mode with a breakpoint inside the Radio callback if statement.
You can check that the code is working by reseting the other DK which will trigger a transmit of the packet.
The packet is 32 bytes long and it's all zeros with the first byte equal to 0x01.
