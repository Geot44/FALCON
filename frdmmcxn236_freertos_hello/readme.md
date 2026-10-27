# freertos_hello

## Overview
The Hello World project is a simple demonstration program that uses the SDK UART driver in
combination with FreeRTOS. The purpose of this demo is to show how to use the debug console and to
provide a simple project for debugging and further development.

The example application creates one task called hello_task. This task print "Hello world." message
via debug console utility and suspend itself.



## Running the demo
After the board is flashed the Tera Term will print "Hello world" message on terminal.

Example output:
Hello world.

## Supported Boards
- [FRDM-MCXN236](#board-specific-information-for-frdmmcxn236)

---

## Board-Specific Information for frdmmcxn236

### Hardware requirements
- Type-C USB cable
- FRDM-MCXN236 board
- Personal Computer

### Board settings
No special settings are required.

### Prepare the Demo
1.  Connect a type-c USB cable between the host PC and the MCU-Link USB port (J10) on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

