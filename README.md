# dacard-firmware-desktop-lib
gamma stage dev repo of dacard software

The main.c file is inside "src" folder.
Any user code must fall between `user code begin <number>` and `user code end <number>`

### Software
The firmware is written entirely in C and the environment used is *Keil MDK v5*

The computer side program is to be written in Python for now, then also including a GUI program, a MATLAB library, and a streamlined python library.

The python program uses **pyserial library** for acquiring data over USB and the same for sending commands.


### References
- Use ARM cortex m3 user guides and books.
- Also look up STM32F103C8Tx datasheet
- Also look at the DACard-Basic repo for visual reference

### Contributing
If you would like to discuss some part of code, create an `issue`
If you would like to contribute some code, create a `pull request`

### Contact

You can contact me directly for now
