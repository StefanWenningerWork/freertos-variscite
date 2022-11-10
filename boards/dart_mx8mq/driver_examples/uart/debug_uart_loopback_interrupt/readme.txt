Overview
========
The debug_uart_loopback_interrupt tries to receive the UART1 TX Stream from the Cortex-A53 via an interrupt
and send it back out via UART3.

Toolchain supported
===================
- IAR embedded Workbench  9.10.2
- GCC ARM Embedded  10.2.1

Hardware requirements
=====================
- Micro USB cable
- DART-MX8MQ SoM
- J-Link Debug Probe
- proper power supply
- Personal Computer

Board settings
==============
No special settings are required.



Prepare the Demo
================
1.  Connect proper power supply and J-Link Debug Probe to the board, switch SW8 to power on the board
2.  Connect a proper cable between the host PC and the UART3 TX, RX and GND on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Launch the debugger in your IDE to begin running the demo.


Running the demo
================
When the demo runs successfully, the log would be seen on the debug terminal like:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Uart functional interrupt example
Board receives characters then sends them out
Now please input:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
