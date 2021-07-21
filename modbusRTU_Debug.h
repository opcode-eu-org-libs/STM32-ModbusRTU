/* Minimalist Modbus RTU implementation for STM32 uC used LibOpenCM3.

   Debug auxiliary functions.


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

#define D_INIT  0x01
#define D_UART  0x02
#define D_TIMER 0x04

#define D_START 0xe0
#define D_END   0xf0

void DEBUG_INFO(uint8_t section, uint8_t info) {
	static uint8_t dinfo = 0;
	
	// set section state in bits 0, 1, 2
	if (info == D_START)
		dinfo |= section;
	else if (info == D_END)
		dinfo &= ~section;
	
	uint16_t port_status = GPIO_ODR(GPIOA) & 0xff00;
	uint16_t dinfo2;
	
	if (info < D_START) {
		dinfo2 = dinfo | (info & 0x0f) << 4;
		gpio_port_write(GPIOA, port_status | dinfo2);
	} else {
		dinfo2 = dinfo | 0x08 | (modbusState & 0x0f) << 4;
		gpio_port_write(GPIOA, port_status | dinfo2);
		__asm__("nop");
		
		dinfo2 = dinfo | (modbus_half_char_counter & 0x0f) << 4;
		gpio_port_write(GPIOA, port_status | dinfo2);
		__asm__("nop");
	}
}

void DEBUG_INIT() {
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 0x00ff);
	
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
	uint16_t port_status = GPIO_ODR(GPIOA) && 0xff00;
	gpio_toggle(GPIOC, GPIO13);
	gpio_port_write(GPIOA, port_status| 0x00);
	for (int i = 0; i < 150000; i++) __asm__("nop");
	for (int j=0; j<7; ++j) {
		gpio_toggle(GPIOC, GPIO13);
		gpio_port_write(GPIOA, port_status| 0x01 << j);
		for (int i = 0; i < 150000; i++) __asm__("nop");
	}
	gpio_toggle(GPIOC, GPIO13);
	gpio_port_write(GPIOA, port_status| 0x00);
	for (int i = 0; i < 150000; i++) __asm__("nop");
	gpio_toggle(GPIOC, GPIO13);
}
