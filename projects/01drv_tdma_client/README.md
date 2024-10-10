# TDMA CLient Example Application

To be flashed onto a nRF52- or nRF53- development Kit, DotBot, or LH2 mini mote.

It initializes a TDMA client and registers to a nearby TDMA server that operates in the same frequency.
Then it sends bursts of 7 messages every 500ms to test the message queueing system.

When used in debug mode, it prints to the terminal client registration status, and the time slot assigned to it.
