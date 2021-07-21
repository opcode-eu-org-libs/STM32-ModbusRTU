/* Minimalist Modbus RTU implementation for STM32 uC used LibOpenCM3.

   Sample RTU Slave application.


For testing you can use mbpool to read / write registers – e.g.:
  - read:
      mbpoll -0v1 -P none -b 9600 -a 2 -t 4:hex -r 0 -c 10  /dev/ttyUSB0
  - write:
      mbpoll -0v1 -P none -b 9600 -a 2 -r 2  /dev/ttyUSB0 0x1713


Copyright (c) 2021, Robert Ryszard Paciorek <rrp@opcode.eu.org>

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
*/

#include "modbusRTU.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/memorymap.h>


// some "registers" for demo use
#define dataBufLen 10
uint16_t dataBuf[dataBufLen] = {0x6601, 0x7702, 0x8803, 0x9904, 0x0005, 0xaa01, 0xbb02, 0xcc03, 0xdd04, 0xee05};


// prepare and mark to send error replay
void sendModbusErrorReplay(uint8_t address, uint8_t function, uint8_t code) {
	initModbusFrame();
	addCharToModbusFrame(address);
	addCharToModbusFrame(function + 0x80);
	addCharToModbusFrame(code);
	sendModbusFrame(1);
}

// frame processing in ISR
uint8_t modbusFrameCallbackISR(uint8_t deviceAddress, uint8_t function, volatile uint8_t *data, uint8_t dataLen) {
	if (deviceAddress == 0x01) { // hight priority frame ... for API usage demonstration
		sendModbusErrorReplay(deviceAddress, function, 0x04);
		return 0;
	}
	// other frames will be process in main loop
	return 1;
}

// frame processing in main loop
void modbusFrameCallback(uint8_t deviceAddress, uint8_t function, volatile uint8_t *data, uint8_t dataLen) {
	if (deviceAddress == 0x02) { // require slave address 0x02, ignore broadcast request
		switch(function) {
			case 0x04:
			case 0x03: {
				uint16_t start = data[0] << 8 | data[1];
				uint16_t count = data[2] << 8 | data[3];
				
				if (dataLen != 4 || count == 0 || count > 0x7d) {
					sendModbusErrorReplay(deviceAddress, function, 0x03); // illegal data value
				} else if (start+count <= dataBufLen) {
					initModbusFrame();
					addCharToModbusFrame(deviceAddress);
					addCharToModbusFrame(function);
					addCharToModbusFrame(2*count);
					count += start;
					for (int i=start; i<count; ++i) {
						addCharToModbusFrame(dataBuf[i] >> 8);
						addCharToModbusFrame(dataBuf[i] & 0xff);
					}
					sendModbusFrame(1); // send replay
				} else {
					sendModbusErrorReplay(deviceAddress, function, 0x02); // illegal data address
				}
				break;
			}
			case 0x06: {
				uint16_t address = data[0] << 8 | data[1];
				if (address < dataBufLen && dataLen == 4) {
					dataBuf[address] = (data[2] << 8) | data[3];
					sendModbusFrame(0); // resend original frame as confirm response
				} else {
					sendModbusErrorReplay(deviceAddress, function, 0x02); // illegal data address
				}
				break;
			}
			default: {
				sendModbusErrorReplay(deviceAddress, function, 0x01); // illegal function
				break;
			}
		}
	}
}

int main() {
	SCB_VTOR = FLASH_BASE;
	initModbusRTU();
	
	while(1) {
		prossessModbusFrame(); // for process frames with deviceAddress != 0x01
	}
}
