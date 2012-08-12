Mozart
======

Mozart is an IR controller for a GE Air Conditioner.
Current implementation is with an Arduino Uno and an Adafruit
Xbee adapter.  IR drivers consists of a PCB from an Adafruit
TV-B-Gone including discrete transistors and IR LEDs but no
microprocessor. Discrete transistors are driven by an Arduino pin, pin3
which is a PWM pin for use by the IR library to toggle the LEDs
at a 40 khz rate.

Xbee addressing: Pan=1234, MY = 0, DL = 1