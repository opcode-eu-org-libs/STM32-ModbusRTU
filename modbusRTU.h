/* Minimalist Modbus RTU implementation for STM32 uC used LibOpenCM3.

   API header.


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

#include <stdint.h>

/**
 * @brief init Modbus RTU slave on UART1 at GPIOA and MODBUS_TIMER timer
 * 
 * @note  modbus, uart and timer configuration via @ref slaveModbusRTU_Config.h file
 */
void initModbusRTU();

/**
 * @brief process modbus frame in main loop
 *
 * @note
 *   1) use @ref modbusFrameCallback callback function for processing frame
 *   2) must be call in main loop unless define @ref MODBUS_USE_CALLBACK_IN_ISR to call @ref modbusFrameCallbackISR in interrupt service routine
 */
void prossessModbusFrame();


/**
 * @brief init modbus frame buffer control variables (including CRC)
 */
void initModbusFrame();

/**
 * @brief add single char to @a modbus_frame_buf with CRC calculation step
 */
void addCharToModbusFrame(uint8_t ch);

/**
 * @brief mark prepared (in @a modbus_frame_buf) modbus frame as ready for sending
 *        when @a addCRC !=0 add CRC to frame
 * 
 * @note 
 *    1) sending will be start automatically via timer call
 *    2) if call in process frame callback function without modify frame (e.g. call @ref initModbusFrame) and with @a addCRC == 0
 *       resend received frame (this is useful as acknowledge of some functions)
 */
void sendModbusFrame(uint8_t addCRC);


/**
 * @brief callback function for received frame, call from @ref prossessModbusFrame
 */
void    modbusFrameCallback(uint8_t deviceAddress, uint8_t function, volatile uint8_t *data, uint8_t dataLen);

/**
 * @brief callback function for received frame, call from timer interrupt service routine
 *
 * @return
 *  when do not call @ref sendModbusFrame and return non zero value then frame processing should be continue in @ref modbusFrameCallback called from @ref prossessModbusFrame
 * 
 * @note
 *    1) must be defined @ref MODBUS_USE_CALLBACK_IN_ISR macro to use this function
 *    2) when used and always return zero using @ref prossessModbusFrame is not necessary
 */
uint8_t modbusFrameCallbackISR(uint8_t deviceAddress, uint8_t function, volatile uint8_t *data, uint8_t dataLen);
