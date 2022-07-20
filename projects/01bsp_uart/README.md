# UART board support package usage example

This application expects an UART to USB cable to be connected on P0.9 (RX)
and P0.10 (TX) pins.
On the computer, use a terminal application to connect to the virtual communication
port (/dev/ttyUSBxx on Linux, COMxx on Windows, etc) created by the UART to USB
cable.

Loading this app onto the DotBot board will echo messages sent from the
computer to the DotBot on the debug terminal.
