LPC-OSC
=======

This is an implementation of the OSC-Embedded library on the LPC11U37 device.

The microcontroller running this code can communicate with computer or another
USB host enabled device, by representing itself as USB CDC device. Host can 
send OSC Messages or Bundles to control various functions of the microcontroller,
including, but not limited to GPIO, Interrupts, PWM, Serial, SPI and I2C.
All available functions and OSC messages that control them are (or will be)
explained in the wiki. OSC Messages and Bundles have to be sent as packets 
over the USB interface. The packet format is also documented in the wiki.

This project has two external dependencies, which should be included when building
the project:
1) OSC-Embedded library - for OSC support (link: https://github.com/GiedriusM/OSC-Embedded)
2) MemoryManager library - for dynamic memory management (link: https://github.com/GiedriusM/MemoryManager)

Alghouth the software was created and tested using only LPC11U37 microcontroller
on the LPCXpresso board, this project should compile and run on other LPC11Uxx
series chips without any modification (except for microcontroller setting).
With a bit more tweaking it should also run on other LPC series microcontrollers.
