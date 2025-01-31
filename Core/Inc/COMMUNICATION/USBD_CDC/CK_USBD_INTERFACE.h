
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "COMMUNICATION/USBD_CDC/usbd_def.h"


void CK_USBD_Init(void);

int CK_USBD_Transmit(void);

int CK_USBD_ReadData(uint8_t* data);

int CK_USBD_WriteRxBuffer(uint8_t* Buf, uint32_t* Len);

int CK_USBD_WriteTxBuffer(uint8_t data);

void CK_USBD_IntPrint(int32_t num);

void CK_USBD_IntPrintln(int32_t num);

void CK_USBD_FloatPrintln(float num);

void CK_USBD_FloatPrint(float num);

void CK_USBD_StringPrintln(const char str[]);

void CK_USBD_StringPrint(const char str[]);



#ifdef __cplusplus
}
#endif

#endif
