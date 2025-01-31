
#ifndef CK_BUZZER_H_
#define CK_BUZZER_H_

#include "stm32h7xx_hal.h"

typedef enum
{

	BUZZER_MODE_DC,
	BUZZER_MODE_PWM

}CK_BUZZER_Mode;

void CK_BUZZER_Init(GPIO_TypeDef* gpio_, uint8_t gpio_pin_, CK_BUZZER_Mode md);

void CK_BUZZER_Activate(void);

void CK_BUZZER_Deactivate(void);

void CK_BUZZER_CheckBuzzer(void);

void CK_BUZZER_Tone1(void);

void CK_BUZZER_Tone2(void);

void CK_BUZZER_Tone3(void);

void CK_BUZZER_Tone4(void);

#endif /* CK_BUZZER_H_ */
