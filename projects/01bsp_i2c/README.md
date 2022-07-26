# I2C board support package usage example

This application suppose that an I2C sensor is connected to the P0.9 (SDA) and
P0.10 (SCL) debug pins of the DotBot. The code is meant to read lsm303agr
accelerometer values.
Loading this app onto the DotBot board will read accelerometer values
and print them in the debug terminal every 200ms.
