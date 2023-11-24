# SailBot IMU driver example

This project contains the IMU driver example dedicated to the SailBot platform.
The example also contains the option to start the calibration procedure, during which the board should be moved in all directions.
The Segger project is configured to save the output debug log to magnetometer.csv file.
When this project is compiled with CALIBRATION_PROCEDURE set, this will dump magnetometer data to the output.
The data can later be plotted using [plot_magnetometer.py](../../dist/scripts/lis2mdl/plot_magnetometer.py) Python script
and the offsets calculated and printed using the [hard_iron.py](../../dist/scripts/lis2mdl/hard_iron.py) script.

