# interface-board
[![Build Status](https://travis-ci.org/MTU-Autobot/interface-board.svg?branch=master)](https://travis-ci.org/MTU-Autobot/interface-board)

Interface board for MTU's IGVC robot. The board connects to an RC controller, wheel encoders, motor controllers, and status lights and is controlled over USB. The microcontroller is a MK20DX256 running a Teensy 3.2 bootloader for easy programming. Main power conversion is handled by a TPS62143 buck converter and can handle up to 17 volts in for compatibility with 12 volt systems.

| <img src="/images/board.png" alt="PCB layout" style="width: 200px;"/> | <img src="/images/pcb.jpg" alt="Assembled PCB" style="width: 300px;"/> |
|-----------------------------------------------------------------------|------------------------------------------------------------------------|

To interface with the board you need to setup the udev rules found [here](https://www.pjrc.com/teensy/loader_linux.html)
