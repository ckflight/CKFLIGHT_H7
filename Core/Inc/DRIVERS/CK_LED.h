
#ifndef CK_LED_H_
#define CK_LED_H_

#include "CK_DEFINITIONS.h"

void CK_LED_Init1(GPIO_TypeDef* gpio_, uint8_t gpio_pin_);

void CK_LED_Init2(GPIO_TypeDef* gpio_, uint8_t gpio_pin_);

void CK_LED_EnableLed(int led_num);

void CK_LED_DisableLed(int led_num);

void CK_LED_ToggleLed(int led_num);

void CK_LED_ToggleLedForMs(int led_num, uint32_t loop, uint32_t time);

#endif /* CK_LED_H_ */
