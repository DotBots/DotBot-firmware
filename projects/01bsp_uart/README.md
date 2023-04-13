# UART board support package usage example

On a DotBot board, this application expects an UART to USB cable to be
connected to P0.9 (RX) and P0.10 (TX) pins. If you want to use this application
on an nRF52840-DK, edit 01bsp_uart.c and apply this patch:
```diff
diff --git a/projects/01bsp_uart/01bsp_uart.c b/projects/01bsp_uart/01bsp_uart.c
index 44aac55..7068b18 100644
--- a/projects/01bsp_uart/01bsp_uart.c
+++ b/projects/01bsp_uart/01bsp_uart.c
@@ -24,8 +24,8 @@ typedef struct {
 
 //=========================== variables ========================================
 
-static const gpio_t _rx_pin   = { .pin = 9, .port = 0 };
-static const gpio_t _tx_pin   = { .pin = 10, .port = 0 };
+static const gpio_t _rx_pin   = { .pin = 8, .port = 0 };
+static const gpio_t _tx_pin   = { .pin = 6, .port = 0 };
 static uart_vars_t _uart_vars = { 0 };
 
 //=========================== callbacks ========================================
```

On the computer, use a terminal application to connect to the virtual communication
port (/dev/ttyUSBxx on Linux, COMxx on Windows, etc) created by the UART to USB
cable. The baudrate is 1Mbaud.

Example with [socat](http://www.dest-unreach.org/socat):

```
socat - open:/dev/ttyACM0,b1000000,echo=0,raw,cs8,parenb=0,cstopb=0
```

Loading this app onto the DotBot board will echo messages sent from the
computer to the DotBot on the debug terminal.
