About this repository
=====================

This is minimal Modbus RTU implemetation for STM32 microcontrollers programed with LibOpenCM3.

* small and simple – less than 400 code lines of "core" library
* support RTU slave (most common case)
* write from scratch for STM32 / LibOpenCM3
* use LibOpenCM3 as HAL
* MIT licence
* all configuration (Modbus, UART, GPIO, Timer) via `#define` in single file `modbusRTU_Config.h`
* processing Modbus PDU in user application code via callback functions
* processing Modbus PDU can be done in ISR or main loop (depending on need and use in application)


## Files

* [modbusRTU.c](modbusRTU.c) – "core" library
* [modbusRTU.h](modbusRTU.h) – API header (exported only function intended to use in user application code)

* [modbusRTU_Config.h](modbusRTU_Config.h) – all configuration settings
* [modbusRTU_Debug.h](modbusRTU_Debug.h) – debug via GPIO (unused until define `MODBUS_DEBUG` macro)
* [modbusCRC.h](modbusCRC.h) – Modbus CRC16 lookup tables

* [main.c](main.c) – sample application


## Build

For build demo app use:

	OPENCM3_DIR=/path/to/libopencm3  make

*libopencm3 must be build first*


## Tested

Tested on STM32F103C8T6.


## References

* Modbus – https://modbus.org/
* LibOpenCM3 – http://libopencm3.org/


## Copyrights

Copyright (c) 2021, Robert Ryszard Paciorek <rrp@opcode.eu.org>,
                    BSD/MIT-type license

This text/program is free document/software. Redistribution and use in
source and binary forms, with or without modification, ARE PERMITTED provided
save this copyright notice. This document/program is distributed WITHOUT any
warranty, use at YOUR own risk.


**The MIT License:**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
