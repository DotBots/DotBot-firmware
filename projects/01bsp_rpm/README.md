# Wheel revolution counter board support package usage example

This application uses the revolution counter driver to print in a loop the right
and left speed (in cm/s), the rotation per minutes and the rotation per seconds.

To be accurate, the values must be read within a period of 125ms which the
sampling period.

## How to get the printed values

To log the values, the application uses SEGGER RTT IO. To access the output, you
need a client application such as SEGGER embedded studio or JLinkRTTViewer (that
comes with JLink programmer).

1. Connect your DotBot to the JTAG of an nrf52840dk board
2. Build and download the `01bsp_rpm` project to the DotBot
3a. If using SEGGER embedded studio, start a debug session (with `F5`) and then
press `F5` a second time to launch the application. The output is available in
the `Debug terminal` frame.
3b. if using JLinkRTTViewer, select `NRF52833_XXAA` as target device and click OK.
The output is displayed in the terminal tab.
