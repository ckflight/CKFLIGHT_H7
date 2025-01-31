
#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"
#include "COMMUNICATION/CK_INPUTSTREAM.h"
#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/CK_MSP.h"

#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_GPIO.h"

/*
 * This class will handle all usb inputs to the flight controller
 * MSP, BLHELIPASSTHROUGH and PRINTER will get input from here.
 */

typedef struct{

	uint8_t input_buffer[512];

	uint16_t input_size;

	bool is_hostMode_active;

}input_stream_t;

input_stream_t inputStream;

input_stream_t inputStream ={

	.input_size = 0,

	.is_hostMode_active = false
};

void CK_INPUTSTREAM_Update(void){

	/*
	 * Input stream class is the general controller of the usb input stream.
	 * It sends the copy of received data to each class that uses usb input command.
	 */
	uint8_t data;
	while(CK_USBD_ReadData(&data) == 1){

		inputStream.input_buffer[inputStream.input_size++] = data;
	}

	if(inputStream.input_size){

		// Send received bytes to each class using input stream.

		CK_PRINTER_DecodeInputStream(inputStream.input_buffer, inputStream.input_size);

		CK_CONFIGURATION_DecodeInputStream(inputStream.input_buffer, inputStream.input_size);

		CK_MSP_DecodeInputStream(inputStream.input_buffer, inputStream.input_size);

		inputStream.input_size = 0;

	}

}



