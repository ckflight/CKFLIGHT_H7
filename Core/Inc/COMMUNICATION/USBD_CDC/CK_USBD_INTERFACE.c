
#include <COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h>
#include "COMMUNICATION/USBD_CDC/usbd_core.h"
#include "COMMUNICATION/USBD_CDC/usbd_desc.h"
#include "COMMUNICATION/USBD_CDC/usbd_cdc.h"
#include "COMMUNICATION/USBD_CDC/usbd_cdc_if.h"

#include "COMMUNICATION/USBD_CDC/CK_USBD_BUFFER.h"

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

#define BUFFER_SIZE		512

uint8_t copy_buffer[BUFFER_SIZE];

uint32_t copy_length;

uint8_t copy_buffer_rx[BUFFER_SIZE];
uint32_t copy_rx_length;

circularBuffer_t usbd_receive_cb;

void CK_USBD_Init(void){
  
	// GPIO is init in usbd_conf.c file void HAL_PCD_MspInit function

	/* Init Device Library, add supported class and start the library. */
	if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_Start(&hUsbDeviceFS) != USBD_OK){
		//Error_Handler();
	}

	/* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
	HAL_PWREx_EnableUSBVoltageDetector();

	CK_USBD_BUFFER_Init(&usbd_receive_cb, BUFFER_SIZE);
  
}

int CK_USBD_Transmit(void){

	if(CDC_Transmit_FS(copy_buffer, copy_length) == USBD_OK){
		copy_length = 0;
		return 1; // OK
	}

	return 0;

}

int CK_USBD_ReadData(uint8_t* data){

	if(!CK_USBD_BUFFER_IsBufferEmpty(&usbd_receive_cb)){
		CK_USBD_BUFFER_BufferRead(&usbd_receive_cb, data);
		return 1;
	}

	return 0; // Data is not available
}

int CK_USBD_WriteRxBuffer(uint8_t* Buf, uint32_t* Len){

	// Later add FAST_TRANSFER mode for reception as well where
	// array should be copied to circular buffer directly once.

	uint32_t receivedLength = *Len; // point to the received length address which is the length number.

	for(int i = 0; i < receivedLength; i++){

		if(!CK_USBD_BUFFER_IsBufferFull(&usbd_receive_cb)){
			uint8_t data = Buf[i];
			CK_USBD_BUFFER_BufferWrite(&usbd_receive_cb, data);
		}
		else{
			return 0; // Full, could not write
		}

	}

	return 1; // OK
}

int CK_USBD_WriteTxBuffer(uint8_t data){

	if(copy_length <= BUFFER_SIZE){
		copy_buffer[copy_length++] = data;
		return 1; // OK
	}

	return 0; // Full, could not write

}

void CK_USBD_IntPrint(int32_t num){
	if(num < 0){
		CK_USBD_WriteTxBuffer((uint8_t)'-');
		num *= -1;
	}
	int32_t tmp[10];
	int numOfDigits = 0;
	int limit = 10;

	for(int i=0; i < limit; i++){
		if(num >= 10){
			tmp[i] = (int32_t)num % 10;
			num = num - tmp[i];
			num = num / 10;
		}
		else{
			tmp[i] = num;
			numOfDigits = i;
			i = limit;
		}
	}
	for(int i = numOfDigits; i >= 0; i--){
		CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
	}
}

void CK_USBD_IntPrintln(int32_t num){
	if(num < 0){
		CK_USBD_WriteTxBuffer((uint8_t)'-');
		num *= -1;
	}
	int32_t tmp[10];
	int numOfDigits = 0;
	int limit = 10;

	for(int i = 0; i < limit; i++){
		if(num >= 10){
			tmp[i] = (int32_t)num % 10;
			num = num - tmp[i];
			num = num / 10;
		}
		else{
			tmp[i] = num;
			numOfDigits = i;
			i = limit;
		}
	}
	for(int i=numOfDigits; i >= 0; i--){
		CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
	}
	CK_USBD_WriteTxBuffer((uint8_t)13);
	CK_USBD_WriteTxBuffer((uint8_t)10); //for ASCII new line

}

void CK_USBD_FloatPrintln(float num){
	int flag = 0;
	num = num * 100; // 2 digit after comma
	int intnum = (int)num;
	const int limit = 10; // 10 digit number max
	if(intnum < 0){
		CK_USBD_WriteTxBuffer((uint8_t)'-');
		intnum *= -1;
	}
	if(intnum < 10){ // 0.00
		flag = 1;
	}
	else if(intnum >= 10 && intnum <= 99){ // 0.01 to 0.99
		flag = 2;
	}

	int tmp[limit];
	int numOfDigits = 0;
	for(int i = 0; i < limit; i++){
		if(intnum>=10){
			tmp[i] = (int)intnum % 10;
			intnum = intnum - tmp[i];
			intnum = intnum / 10;
		}
		else{
			tmp[i] = intnum;
			numOfDigits = i; // get num of digits
			i = limit; // end loop
		}
	}
	if(flag==1){ // make 0.00
		tmp[1] = 0;tmp[2] = 0;
		numOfDigits = 2;
	}
	if(flag==2){ // add 0 for 0.01 to 0.99 numbers
		tmp[++numOfDigits] = 0;
	}

	for(int i=numOfDigits; i >= 0; i--){
		if(i>1){
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else if(i==1){
			CK_USBD_WriteTxBuffer((uint8_t)46); //for ASCII '.'
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else{
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
	}
	CK_USBD_WriteTxBuffer((uint8_t)13);
	CK_USBD_WriteTxBuffer((uint8_t)10); //for ASCII new line

}

void CK_USBD_FloatPrint(float num){
	int flag = 0;
	num = num * 100; // 2 digit after comma
	int intnum = (int)num;
	const int limit = 10; // 10 digit number max
	if(intnum < 0){
		CK_USBD_WriteTxBuffer((uint8_t)'-');
		intnum *= -1;
	}
	if(intnum < 10){ // 0.00
		flag = 1;
	}
	else if(intnum >= 10 && intnum <= 99){ // 0.01 to 0.99
		flag = 2;
	}

	int tmp[limit];
	int numOfDigits = 0;
	for(int i = 0; i < limit; i++){
		if(intnum >= 10){
			tmp[i] = (int)intnum % 10;
			intnum = intnum - tmp[i];
			intnum = intnum / 10;
		}
		else{
			tmp[i] = intnum;
			numOfDigits = i; // get num of digits
			i = limit; // end loop
		}
	}
	if(flag == 1){ // make 0.00
		tmp[1] = 0;tmp[2] = 0;
		numOfDigits = 2;
	}
	if(flag == 2){ // add 0 for 0.01 to 0.99 numbers
		tmp[++numOfDigits] = 0;
	}

	for(int i = numOfDigits; i >= 0; i--){
		if(i > 1){
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else if(i == 1){
			CK_USBD_WriteTxBuffer((uint8_t)46); //for ASCII '.'
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else{
			CK_USBD_WriteTxBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
	}

}

void CK_USBD_StringPrintln(const char str[]){
	int size = strlen(str);
	for(int i = 0; i < size; i++){
		CK_USBD_WriteTxBuffer((uint8_t)str[i]);
	}
	CK_USBD_WriteTxBuffer((uint8_t)13);
	CK_USBD_WriteTxBuffer((uint8_t)10); //for ASCII new line

}

void CK_USBD_StringPrint(const char str[]){
	int size = strlen(str);
	for(int i = 0; i < size; i++){
		CK_USBD_WriteTxBuffer((uint8_t)str[i]);
	}

}
