# AS5048B rotary encoder driver usage example

Loading this app onto the SailBot board will read the encoder's raw output every 50 milliseconds in an infinite loop.
The delay between read function calls is not strictly necessary. The read function can be called sequentially with no instructions in between, with no error whatsoever.