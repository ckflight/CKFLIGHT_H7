#pragma once

#include <stdbool.h>
#include <stdint.h>

void  CK_DPS310_Init(I2C_TypeDef* I2Cn, uint32_t baroFreq);

void  CK_DPS310_ReadBaroRaw(void);
void  CK_DPS310_ReadBaro(void);

float CK_DPS310_GetPressurePa(void);
float CK_DPS310_GetTemperatureC(void);

bool  CK_DPS310_isBaroSensorInitialized(void);
