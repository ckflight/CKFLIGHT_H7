
#ifndef CK_BLHELIPASS_H_
#define CK_BLHELIPASS_H_

#include "CK_DEFINITIONS.h"

void CK_BLHELIPASS_Init(void);

void CK_BLHELIPASS_ESC1_Passthrough(void);

void CK_BLHELIPASS_ESC2_Passthrough(void);

void CK_BLHELIPASS_ESC3_Passthrough(void);

void CK_BLHELIPASS_ESC4_Passthrough(void);

void CK_BLHELIPASS_Set_ONEWIRE_Output(GPIO_TypeDef* GPIOx, uint8_t pin);

void CK_BLHELIPASS_Set_ONEWIRE_Input(GPIO_TypeDef* GPIOx, uint8_t pin);

void CK_BLHELIPASS_ONEWIRE_PULLUP(GPIO_TypeDef* GPIOx, uint8_t pin);

void CK_BLHELIPASS_ONEWIRE_CLEARPULLUP(GPIO_TypeDef* GPIOx, uint8_t pin);

void CK_BLHELIPASS_ApplyUSBCommand(void);

void CK_BLHELIPASS_ReadUSBCommand(void);

#endif /* CK_BLHELIPASS_H_ */
