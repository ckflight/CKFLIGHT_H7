
#ifndef INC_CK_I2C_H_
#define INC_CK_I2C_H_

#include "CK_DEFINITIONS.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

typedef enum{

	USE_HAL_I2C,
	USE_CK_I2C

}CK_I2C_LIB;

typedef enum{

	CK_I2C_Transmit,
	CK_I2C_Receive

}CK_I2C_Mode;

typedef enum{

	CK_I2C_ACKDisable,
	CK_I2C_ACKEnable

}CK_I2C_ACK_Mode;

typedef enum{

	CK_I2C_100Khz,
	CK_I2C_400Khz

}CK_I2C_Speed;

void CK_I2C_Init(I2C_TypeDef* i2c, CK_I2C_Speed freq, CK_I2C_LIB lib);

uint16_t CK_I2C_IsBusy(I2C_TypeDef* I2Cx);

void CK_I2C_Transfer(I2C_TypeDef* i2c_, uint8_t slaveAddress, uint8_t reg, uint8_t data);

void CK_I2C_ReadMultiInterrupt(I2C_HandleTypeDef* i2c_handler, uint8_t slaveAddress, uint8_t reg, uint8_t* rxBuffer, int quantity);

void CK_I2C_ReadMulti(I2C_TypeDef* i2c_, uint8_t slaveAddress, uint8_t reg, uint8_t* rxBuffer, int quantity);

void CK_I2C_Start(I2C_TypeDef* I2Cx, uint8_t slaveAddress, CK_I2C_Mode mode, CK_I2C_ACK_Mode ack);

void CK_I2C_Stop(I2C_TypeDef* I2Cx);

uint8_t CK_I2C_ReadAck(I2C_TypeDef* I2Cx);

uint8_t CK_I2C_ReadNack(I2C_TypeDef* I2Cx);

ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

int CK_I2C_CheckInitialized(I2C_TypeDef* I2Cn);

void CK_I2C_TimeOutCounter(I2C_TypeDef* I2Cn);

uint32_t CK_I2C_GetTimeOut(I2C_TypeDef* I2Cn);

void CK_I2C_ResetTimeOut(I2C_TypeDef* I2Cn);

// Compute SCLDEL, SDADEL, SCLH and SCLL for TIMINGR register according to reference manuals.
void CK_I2C_ClockComputeRaw(uint32_t pclkFreq, int i2cFreqKhz, int presc, int dfcoeff, uint8_t *scldel, uint8_t *sdadel, uint16_t *sclh, uint16_t *scll);

uint32_t CK_I2C_ClockTIMINGR(uint32_t pclkFreq, int i2cFreqKhz, int dfcoeff);

#endif /* INC_CK_I2C_H_ */
