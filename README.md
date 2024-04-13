# PiGPIO-Zig

PiGPIO-Zig is a module that communicates with the Raspberry Pi's pigpiod
GPIO daemon.

~~PiGPIO-Zig provides most of the capabilities of C based pigpiod_if2
interface, but is written entirely in zig.  The missing capabilities are
functions that pigpiod implements directly on the client, and that Zig
provides native function for.~~

The above paragraph represents the goal, not the reality.

Currently working:

- Pin set value, direction, pull up and down
- Pin block get, set and clear
- Pin level change callback
- Event callback
- SPI master communications
- Get tick
- Get hardware version
- Get pigpiod version

Present but not well tested

- Pin PWM functions
- Pin Servo functions
- Pin glitch and noise filters
- I2C master communications
- Serial interface (hardware and bit-bang)
- Scripting commands
- File operation commands

If the current version does not support something you need, send a message via
[the Curuvar contact page](https://curuvar.com/contact?to=support).


## A note on formatting

I've been programming computers since 1968.  In that time I have developed
a coding style that I am comfortable with.  I have nothing against others
choosing a different style, just don't push it on me.

If you with to use this code in your own project, the license allows you to
reformat it and you may do so with my blessing.  I have no desire to force
my style on you; however, if you wish to contribute to this project, please
respect the style that I have chosen:

- Allman style braces.
- Single space inside parentheses for function definition and calls.
- When breaking long lines at a binary operator, the operator goes on
  the start of the new line.
- Align parts of similar statements vertically.
- Function parameters (except for self) start with:
  - in_  -- data passed into a function
  - out_ -- data passed out from a function
  - io_  -- data passed into a function and modifications passed out
