# Ultrasound ranging board support package usage example using HC-SR04 sensor on the DotBot board


This is a short example of how to do Ultrasound ranging with HC-SR04 sensor on the DotBot board.
A trigger pulse for the US ranging is sent every 200 ms with the duration of 10 us. 
The US range is captured on the US read pin where the pulse duration corresponds to a distance measured.
We just need to divide the reading in us with 57 to get the distance to an object in mm. 

This code currently works on nRF52840. To use the code on DotBot you need to define different GPIO for US sensor ON and READ pin (trigger and echo) in hc_sr04.c

The US ranging code is developed using GPIOTE, TIMERs and PPI. 

- The GPIOTE is configured with three channels 
	- Channel 0 for triggering the US (toggling the Trigger pin on HC-SR04 from Low to High)
	- Channel 1 for the US read pin to create an event when there is a rising edge
	- Channel 2 for the US read pin to create an event when there is a falling edge
	
- TIMERs:
	- TIMER0 is used to control the trigger pulse (COMPARE[0] register is set with the pulse offset (200 ms) and COMPARE[1] with the pulse duration (10 us))
	- TIMER1 is used to capture the pulse duration on the US read pin, which corresponds to the distance

- PPI chains the TIMER events and GPIOTE tasks to start ranging 

- The result of the US ranging is passed in the us_callback as the us_reading parameter.





