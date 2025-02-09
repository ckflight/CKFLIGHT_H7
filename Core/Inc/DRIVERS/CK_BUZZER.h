
#ifndef CK_BUZZER_H_
#define CK_BUZZER_H_

#include "CK_DEFINITIONS.h"

typedef enum{

	BUZZER_MODE_DC,
	BUZZER_MODE_PWM

}buzzer_mode_e;

void CK_BUZZER_Init(GPIO_TypeDef* gpio_, uint8_t gpio_pin_, buzzer_mode_e md);

void CK_BUZZER_Toggle(void);

void CK_BUZZER_Enable(void);

void CK_BUZZER_Disable(void);

void CK_BUZZER_CheckBuzzer(void);

void CK_BUZZER_Tone1(void);

void CK_BUZZER_Tone2(void);

void CK_BUZZER_Tone3(void);

void CK_BUZZER_Tone4(void);

#endif /* CK_BUZZER_H_ */
