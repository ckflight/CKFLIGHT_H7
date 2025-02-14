/*
 * mpu6000.h
 *
 *  Created on: Feb 13, 2025
 *      Author: ck
 */

#ifndef INC_SENSORS_MPU6000_H_
#define INC_SENSORS_MPU6000_H_

uint8_t CK_MPU6000_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq);
bool CK_MPU6000_isGyroSensorInitialized(void);
bool CK_MPU6000_isAccSensorInitialized(void);

#endif /* INC_SENSORS_MPU6000_H_ */
