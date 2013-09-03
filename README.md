LPC-OSC
=======

This is an implementation of the OSC-Embedded library for the LPC11U37 device.

The microcontroller running this code can communicate with a computer or another
USB host enabled device, by representing itself as USB CDC device. The host can 
send OSC Messages or Bundles to control various functions of the microcontroller,
including, but not limited to, GPIO, Interrupts, PWM, Serial, SPI and I2C.
All available functions and OSC messages that control them are (or will be)
explained in the wiki. OSC Messages and Bundles have to be sent as packets 
over the USB interface. The packet format is also documented in the wiki.

This project has two external dependencies, which should be included when building
the project:
1) OSC-Embedded library - for OSC support (https://github.com/8devices/OSC-Embedded)
2) MemoryManager library - for dynamic memory management (https://github.com/8devices/MemoryManager)

Althouth the software was created and tested using only the LPC11U37 microcontroller
on the LPCXpresso board, this project should compile and run on the other LPC11Uxx 
series chips without any modification (except for microcontroller setting).
With a bit more tweaking it should also run on the other LPC series microcontrollers.
