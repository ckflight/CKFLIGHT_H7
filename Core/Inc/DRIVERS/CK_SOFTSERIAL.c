
#include "DRIVERS/CK_SOFTSERIAL.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"

#define clockCyclesToMicroseconds(a)  (((a) * 1000L) / (F_CPU / 1000L))

#define cycle	0

uint32_t _bitPeriod = 0;
uint8_t _stop_bit = 0;

uint8_t _start_bit_ = 0;
uint8_t _idle_bit_ = 0;

uint8_t _read_bit_num = 8;

circularBuffer_t softserial_cb;

GPIO_TypeDef* CK_SMART_AUDIO_GPIO;

uint8_t CK_SMART_AUDIO_GPIO_PIN;

void CK_SOFTSERIAL_Init(uint32_t baudRate, uint8_t stop_bit, uart_idle_polarity_t polarity){

	// Works with 115200 like baudrate but cannot send 0x00
	// 4800 baudrate sends them without problem

	#if SMART_AUDIO_
	CK_SMART_AUDIO_GPIO = SMART_AUDIO_GPIO;
	CK_SMART_AUDIO_GPIO_PIN = SMART_AUDIO_GPIO_PIN;
	#endif

	if(polarity == IDLE_HIGH){
		_start_bit_ = 0;
		_idle_bit_ = 1;
	}
	else if(polarity == IDLE_LOW){
		_start_bit_ = 1;
		_idle_bit_ = 0;
	}

	_bitPeriod = 1000000L / baudRate;

	_stop_bit = stop_bit;

	// For testing
	CK_GPIO_Init(GPIOA, 10, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_PULLUP); //RX

}

void CK_SOFTSERIAL_SetOutput(void){

	CK_GPIO_ClockDisable(CK_SMART_AUDIO_GPIO);

	CK_GPIO_DeInit(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN);

	CK_GPIO_ClockEnable(CK_SMART_AUDIO_GPIO);

	CK_GPIO_Init(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

    CK_GPIO_SetPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN);

}

void CK_SOFTSERIAL_SetInput(void){

	CK_GPIO_ClockDisable(CK_SMART_AUDIO_GPIO);

	CK_GPIO_DeInit(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN);

	CK_TIME_DelayMilliSec(10);

	CK_GPIO_ClockEnable(CK_SMART_AUDIO_GPIO);

	CK_GPIO_Init(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN, CK_GPIO_INPUT, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

}

bool CK_SOFTSERIAL_IsAvailable(void){

	return !CK_CIRCULARBUFFER_IsBufferEmpty(&softserial_cb);
}

int CK_SOFTSERIAL_Read(void){

	uint8_t val = 0;
	int bitDelay = _bitPeriod - clockCyclesToMicroseconds(cycle);

	uint32_t timeout = CK_TIME_GetMilliSec();

	// one byte of serial data (LSB first)
	// ...--\    /--\/--\/--\/--\/--\/--\/--\/--\/--...
	//	 \--/\--/\--/\--/\--/\--/\--/\--/\--/
	//	start  0   1   2   3   4   5   6   7 stop

	// Wait while pin is 1 for 100ms
	// Response should be received within 100ms
	while((CK_GPIO_ReadPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN) == _idle_bit_) && ((CK_TIME_GetMilliSec() - timeout) < 110));

	// confirm that this is a real start bit, not line noise
	if (CK_GPIO_ReadPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN) == _start_bit_) {

		// frame start indicated by a falling edge and low start bit
		// jump to the middle of the low start bit
		CK_TIME_DelayMicroSec(bitDelay / 2 - clockCyclesToMicroseconds(cycle));

		// offset of the bit in the byte: from 0 (LSB) to 7 (MSB)
		for (int offset = 0; offset < _read_bit_num; offset++) {

			// jump to middle of next bit
			CK_TIME_DelayMicroSec(bitDelay);

			// read bit
			val |= CK_GPIO_ReadPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN) << offset;

		}

		CK_TIME_DelayMicroSec(_bitPeriod);

	}

	CK_CIRCULARBUFFER_BufferRead(&softserial_cb, &val);

	return val;


}

void CK_SOFTSERIAL_Write(uint8_t b){

	int bitDelay = _bitPeriod - clockCyclesToMicroseconds(cycle); // a digitalWrite is about 50 cycles
	uint8_t mask;

	CK_GPIO_ClearPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN); // send 0
	CK_TIME_DelayMicroSec(bitDelay);

	for (mask = 0x01; mask; mask <<= 1) {
		if (b & mask){ // choose bit
			CK_GPIO_SetPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN); // send 1
		}
		else{
			CK_GPIO_ClearPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN); // send 0
		}
		CK_TIME_DelayMicroSec(bitDelay);
	}

	CK_GPIO_SetPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN); // send 1
	CK_TIME_DelayMicroSec(bitDelay);

	if(_stop_bit == 2){
		CK_GPIO_SetPin(CK_SMART_AUDIO_GPIO, CK_SMART_AUDIO_GPIO_PIN); // send 1
		CK_TIME_DelayMicroSec(bitDelay);
	}


}









