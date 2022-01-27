# Motors usage example.

## Introduction

<p align="center">
  <img src="./../../static/motors_demo.gif" alt="motors stand-alone demo"/>
</p>

Loading this example into the DotBot board will cause right motor to spin forward for 2sec and then backwards for 2sec, followed by the left motor doing the same. As it can be seen in the above video.

The DotBot uses a DRV8833 dual H-bridge driver with a 4 pmw control interface. the PWM0 peripheral is used to generate the pwm signals it requires.