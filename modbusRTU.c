/* Minimalist Modbus RTU implementation for STM32 uC used LibOpenCM3

   Modbus RTU microcontroller code.


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
#include "modbusCRC.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

/// maximum modbus frame length
#define MODBUS_FRAME_BUF_SIZE 256

/// space between two modbus frames in multiples of modbus_half_char_counter
#define MODBUS_FRAME_SPACE 7

/// broadcast modbus slave address
#define MODBUS_BROADCAST_ADDRESS 0

#include "modbusRTU_Config.h"

/// received modbus frame buffer
volatile uint8_t  modbus_frame_buf[MODBUS_FRAME_BUF_SIZE];

/// current position in modbus_frame_buf
volatile uint16_t modbus_frame_pos;

/// current position in modbus_frame_buf
volatile uint16_t modbus_data_for_send_lenght;

/// number of "half modbus char" since last restartModbusTimer() call
volatile uint8_t  modbus_half_char_counter;

/// state of modbus subsystem
enum {
	MODBUS_RECEIVING_FRAME          = 1, ///< wait for frame begin or for next char in frame or for frame end
	MODBUS_FRAME_INVALID            = 2, ///< current frame is corrupted (bad timing, too long, etc) and will be ignored
	MODBUS_PARSE_FRAME              = 3, ///< frame is parsing, don't revcive or send any data
	MODBUS_RESPONSE_IS_READY        = 4, ///< data is ready to send, but send not started
	MODBUS_SENDING_RESPONSE         = 5, ///< send data is in progress (load data to UART buffer)
	MODBUS_SENDING_RESPONSE_FINISH  = 6, ///< send data is in progress (all data in UART buffer)
} modbusState;

#ifndef HW_CRC
/// modbus CRC first byte
volatile uint8_t crcL;
/// modbus CRC second byte
volatile uint8_t crcH;
#endif

#ifdef MODBUS_DEBUG
	#include "modbusRTU_Debug.h"
#else
	#define DEBUG_INFO(a, b)
	#define DEBUG_INIT()
#endif

void initModbusFrame() {
	crcL = 0xff;
	crcH = 0xff;
	modbus_frame_pos = 0;
}

void addCharToModbusFrame(uint8_t ch) {
	#ifndef HW_CRC
	uint8_t idx = crcL ^ ch;
	crcL = crcH ^ auchCRCHi[idx];
	crcH = auchCRCLo[idx];
	#endif
	modbus_frame_buf[modbus_frame_pos++] = ch;
}

void sendModbusFrame(uint8_t addCRC) {
	if(addCRC) {
		// add CRC to frame
		#ifndef HW_CRC
		modbus_frame_buf[modbus_frame_pos++] = crcL;
		modbus_frame_buf[modbus_frame_pos++] = crcH;
		#endif
	}
	modbusState = MODBUS_RESPONSE_IS_READY;
}

/// restart "half modbus char" timer, can be called without "stop" before
void restartModbusTimer() {
	timer_disable_counter(MODBUS_TIMER);
	timer_set_counter(MODBUS_TIMER, 0);
	timer_clear_flag(MODBUS_TIMER, TIM_SR_UIF);
	modbus_half_char_counter = 0;
	timer_enable_irq(MODBUS_TIMER, TIM_DIER_UIE);
	timer_enable_counter(MODBUS_TIMER);
}

/// stop "half modbus char" timer
void stopModbusTimer() {
	modbus_half_char_counter = 0;
	timer_disable_irq(MODBUS_TIMER, TIM_DIER_UIE);
	timer_disable_counter(MODBUS_TIMER);
}

void prossessModbusFrame() {
	if (modbusState == MODBUS_PARSE_FRAME) {
		modbusFrameCallback(modbus_frame_buf[0], modbus_frame_buf[1], modbus_frame_buf+2, modbus_frame_pos-4);
		if (modbusState != MODBUS_RESPONSE_IS_READY) {
			stopModbusTimer();
			initModbusFrame();
			modbusState = MODBUS_RECEIVING_FRAME;
		}
	}
}

void initModbusRTU() {
	DEBUG_INIT();
	DEBUG_INFO(D_INIT, D_START);
	
	// Setup modbus internal state
	initModbusFrame();
	modbusState = MODBUS_FRAME_INVALID; // start in out of frame state ... wait for 3.5 chars silence
	
	// Setup timer - 2 ticks per modbus char
	// -> calculate period time (in us) depending on baud rate
	uint32_t half_char_time;
	if (MODBUS_BAUD_RATE > 19200) {
		// half char time for modbus is 250us
		half_char_time = 250UL;
	} else {
		// half char time for modbus is 1000000/(MODBUS_BAUD_RATE/10)/2
		half_char_time = 5000000UL/MODBUS_BAUD_RATE;
	}
	// -> enable clock, interrupts and reset
	rcc_periph_clock_enable(MODBUS_TIMER_RCC);
	nvic_enable_irq(MODBUS_TIMER_IRQ);
	rcc_periph_reset_pulse(MODBUS_TIMER_RST);
	// -> configure timer:
	//    * standard (TIM_CR1_CMS_EDGE) up (TIM_CR1_DIR_UP) counting
	//    * prescaler = 32, no multipler (TIM_CR1_CKD_CK_INT) => one tick for 4 us
	//    * period = half_char_time/4 (ticks) = half_char_time us
	timer_set_mode(MODBUS_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(MODBUS_TIMER, 32 - 1);
	timer_set_period(MODBUS_TIMER, half_char_time/4 - 1);
	
	// Setup GPIOs for UART
	gpio_set_mode(MODBUS_UART_TX_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MODBUS_UART_TX_PIN);
	gpio_set_mode(MODBUS_UART_RX_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, MODBUS_UART_RX_PIN);
	#if defined(MODBUS_UART_USE_TXEN) && MODBUS_UART_USE_TXEN > 0
	gpio_set_mode(MODBUS_UART_TXEN_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MODBUS_UART_TXEN_PIN);
	#if MODBUS_UART_USE_TXEN == 1
		gpio_clear(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
	#elif MODBUS_UART_USE_TXEN == 2
		gpio_set(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
	#endif
	#endif
	
	// Setup UART parameters
	usart_set_baudrate(MODBUS_UART, MODBUS_BAUD_RATE);
	usart_set_databits(MODBUS_UART, 8);
	usart_set_stopbits(MODBUS_UART, USART_STOPBITS_1);
	usart_set_parity(MODBUS_UART, USART_PARITY_NONE);
	usart_set_flow_control(MODBUS_UART, USART_FLOWCONTROL_NONE);
	usart_set_mode(MODBUS_UART, USART_MODE_TX_RX);
	
	// Enable UART interrupts
	nvic_enable_irq(MODBUS_UART_IRQ);
	usart_enable_rx_interrupt(MODBUS_UART);
	
	// Start timer
	restartModbusTimer();
	
	DEBUG_INFO(D_INIT, D_END);
}

/**
 * @brief "half modbus char" timer interrupt
 */
void MODBUS_TIMER_ISR(void) {
	DEBUG_INFO(D_TIMER, D_START);
	
	if (timer_get_flag(MODBUS_TIMER, TIM_SR_UIF)) {
		timer_clear_flag(MODBUS_TIMER, TIM_SR_UIF);
		
		// increase "half modbus char" counter
		++modbus_half_char_counter;
		
		// if silence is greater / equal 3.5 chars we received frame
		if (modbus_half_char_counter >= MODBUS_FRAME_SPACE) {
			switch (modbusState) {
				case MODBUS_RECEIVING_FRAME:
					#ifdef MODBUS_DEBUG
					gpio_toggle(GPIOC, GPIO13);
					#endif
					// frame length must be greater / equal 4 (address, function, 2 byte crc)
					// frame must start from correct slave address or broadcast address
					// frame must have correct crc
					if (
						modbus_frame_pos >= 4 &&
						#ifdef MODBUS_SLAVE_ADDRESS
						(modbus_frame_buf[0] == MODBUS_SLAVE_ADDRESS || modbus_frame_buf[0] == MODBUS_BROADCAST_ADDRESS) &&
						#endif
						#ifndef HW_CRC
						crcL == 0 && crcH == 0
						#endif
					) {
						modbusState = MODBUS_PARSE_FRAME;
						#ifdef MODBUS_USE_CALLBACK_IN_ISR
						uint8_t continue_frame_parsing = modbusFrameCallbackISR(modbus_frame_buf[0], modbus_frame_buf[1], modbus_frame_buf+2, modbus_frame_pos-4);
						if (modbusState == MODBUS_RESPONSE_IS_READY || continue_frame_parsing)
							break;
						#else
						break;
						#endif
					}
					// otherwise it is invalid frame ... so go to next case
					__attribute__ ((fallthrough));
				case MODBUS_FRAME_INVALID:
					// end receiving corrupted frame ... init for next frame
					stopModbusTimer();
					initModbusFrame();
					modbusState = MODBUS_RECEIVING_FRAME;
					break;
				case MODBUS_RESPONSE_IS_READY:
					// response will be send on next MODBUS_TIMER_ISR call – 4.0 (not 3.5) chars space
					// due to inaccuracy with 3.5 char time calculation
					
					// stop timer
					stopModbusTimer();
					
					// disable RX
					usart_disable_rx_interrupt(MODBUS_UART);
					
					modbus_data_for_send_lenght = modbus_frame_pos;
					modbus_frame_pos = 0;
					modbusState = MODBUS_SENDING_RESPONSE;
					
					// enable RS485 transmitter
					#if defined(MODBUS_UART_USE_TXEN)
						#if MODBUS_UART_USE_TXEN == 1
							gpio_set(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
						#elif MODBUS_UART_USE_TXEN == 2
							gpio_clear(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
						#endif
					#endif
					
					// enable TX → start sending frame
					usart_enable_tx_interrupt(MODBUS_UART);
					break;
				default:
					break;
			}
		}
	}
	
	DEBUG_INFO(D_TIMER, D_END);
}


/**
 * @brief usart get char / send next char interrupt
 */
void MODBUS_UART_ISR(void) {
	DEBUG_INFO(D_UART, D_START);
	
	uint32_t flags = USART_SR(MODBUS_UART);
	
	// when receiver buffer not empty
	if ( flags & USART_SR_RXNE ) {
		// receive char (always, even when ignored)
		uint8_t recvChar = usart_recv(MODBUS_UART);
		
		if (modbusState == MODBUS_RECEIVING_FRAME) {
			if (modbus_half_char_counter > 3) {
				// char too late
				modbusState = MODBUS_FRAME_INVALID;
			} else if (modbus_frame_pos >= MODBUS_FRAME_BUF_SIZE) {
				// out of frame
				modbusState = MODBUS_FRAME_INVALID;
			} else {
				// add char to received frame buffer
				addCharToModbusFrame(recvChar);
			}
		}
		
		// restart timer (always, even when receive char was ignored)
		restartModbusTimer();
	}
	
	// when transmit buffer empty and we in sending state
	if ( (flags & USART_SR_TXE) && (modbusState == MODBUS_SENDING_RESPONSE) ) {
		if (modbus_frame_pos < modbus_data_for_send_lenght) {
			// send (write to out buffer) one char
			usart_send(MODBUS_UART, modbus_frame_buf[modbus_frame_pos++]);
		} else {
			// no more chars to send, so:
			//  - disable TX empty interrupt
			usart_disable_tx_interrupt(MODBUS_UART);
			// - wait to transmit complete
			modbusState = MODBUS_SENDING_RESPONSE_FINISH;
			usart_enable_tx_complete_interrupt(MODBUS_UART);
		}
	}
	
	// when transmit complete and we in sending state – switch to receiving, so:
	if ( (flags & USART_SR_TC) && (modbusState == MODBUS_SENDING_RESPONSE_FINISH) ) {
		//  - disable RS485 transmitter
		#if defined(MODBUS_UART_USE_TXEN)
			#if MODBUS_UART_USE_TXEN == 1
				gpio_clear(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
			#elif MODBUS_UART_USE_TXEN == 2
				gpio_set(MODBUS_UART_TXEN_PORT, MODBUS_UART_TXEN_PIN);
			#endif
		#endif
		//  - disable transmit complete interrupt
		usart_disable_tx_complete_interrupt(MODBUS_UART);
		//  - start in out of frame state ... wait for 3.5 chars silence
		modbusState = MODBUS_FRAME_INVALID;
		restartModbusTimer();
		//  - enable RX
		usart_enable_rx_interrupt(MODBUS_UART);
	}
	
	DEBUG_INFO(D_UART, D_END);
}
