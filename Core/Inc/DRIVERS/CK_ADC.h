
#ifndef INC_DRIVERS_CK_ADC_H_
#define INC_DRIVERS_CK_ADC_H_

#include "CK_DEFINITIONS.h"

void CK_ADC_Init(uint32_t adcT, uint32_t mainT);

void CK_ADC_ADC1Init(void);

void CK_ADC_ADC2Init(void);

void CK_ADC_ADC3Init(void);

void CK_ADC_Update(void);

float CK_ADC_GetLipoResult(void);

float CK_ADC_GetCurrentResult(void);

float CK_ADC_GetTemperatureResult(void);

void CK_ADC_CalculateTemperature(void);

void CK_ADC_StartConversion(void);



#endif /* INC_DRIVERS_CK_ADC_H_ */
