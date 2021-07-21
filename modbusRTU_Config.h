/* Minimalist Modbus RTU implementation for STM32 uC used LibOpenCM3.

   Configuration header.


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


/// enable modbus debug on PA0..7 and PC13
#define MODBUS_DEBUG
#undef MODBUS_DEBUG

/// slave modbus device address, when not define receive all frames
#define MODBUS_SLAVE_ADDRESS 2
#undef  MODBUS_SLAVE_ADDRESS

/// use @ref modbusFrameCallbackISR as frame processing in timer interrupt service routine
#define CALLBACK_IN_ISR

/**
 * @name UART configuration
 *
 * @}
 */

#define MODBUS_BAUD_RATE     9600

#define MODBUS_UART          USART1
#define MODBUS_UART_IRQ      NVIC_USART1_IRQ
#define MODBUS_UART_ISR      usart1_isr

#define MODBUS_UART_RX_PORT    GPIOA
#define MODBUS_UART_RX_PIN     GPIO10

#define MODBUS_UART_TX_PORT    GPIOA
#define MODBUS_UART_TX_PIN     GPIO9

/// when MODBUS_UART_USE_TXEN define to 1 use MODBUS_UART_TXEN_PORT/MODBUS_UART_TXEN_PIN to enable RS485 transmitter (pin in high state when transmit)
/// when MODBUS_UART_USE_TXEN define to 2 use MODBUS_UART_TXEN_PORT/MODBUS_UART_TXEN_PIN to enable RS485 transmitter (pin in low  state when transmit)
#define MODBUS_UART_USE_TXEN   1
#define MODBUS_UART_TXEN_PORT  GPIOA
#define MODBUS_UART_TXEN_PIN   GPIO8

/**
 * @}
 *
 * @name timer configuration
 *
 * @}
 */

#define MODBUS_TIMER         TIM3
#define MODBUS_TIMER_RCC     RCC_TIM3
#define MODBUS_TIMER_IRQ     NVIC_TIM3_IRQ
#define MODBUS_TIMER_RST     RST_TIM3
#define MODBUS_TIMER_ISR     tim3_isr

/**
 * @}
 */
