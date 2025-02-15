/*
 * mpu6000.h
 *
 *  Created on: Feb 13, 2025
 *      Author: ck
 */

#ifndef INC_SENSORS_MPU6000_H_
#define INC_SENSORS_MPU6000_H_

uint8_t CK_MPU6000_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);
uint8_t CK_MPU6000_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq);
void CK_MPU6000_AlignGyro(int x, int y, int z);
void CK_MPU6000_AlignAcc(int x, int y, int z);
void CK_MPU6000_ReadGyroRaw(void);
void CK_MPU6000_ReadAccRaw(void);
void CK_MPU6000_ReadSensorRaw_DMA(void);
void SENSOR_DMA_TX_Handler(void);
void SENSOR_DMA_RX_Handler(void);
float CK_MPU6000_ReadTempRaw(void);
bool CK_MPU6000_isGyroSensorInitialized(void);
bool CK_MPU6000_isAccSensorInitialized(void);


















#endif /* INC_SENSORS_MPU6000_H_ */
