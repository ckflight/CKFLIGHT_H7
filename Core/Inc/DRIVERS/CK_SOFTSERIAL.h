
#ifndef INC_DRIVERS_CK_SOFTSERIAL_H_
#define INC_DRIVERS_CK_SOFTSERIAL_H_

#include "CK_DEFINITIONS.h"

typedef enum {
  IDLE_LOW,
  IDLE_HIGH

}uart_idle_polarity_t;

void CK_SOFTSERIAL_Init(uint32_t baudRate, uint8_t stop_bit, uart_idle_polarity_t polarity);

void CK_SOFTSERIAL_SetOutput(void);

void CK_SOFTSERIAL_SetInput(void);

bool CK_SOFTSERIAL_IsAvailable(void);

int CK_SOFTSERIAL_Read(void);

void CK_SOFTSERIAL_Write(uint8_t b);

#endif /* INC_DRIVERS_CK_SOFTSERIAL_H_ */
