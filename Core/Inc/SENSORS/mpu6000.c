/*
 * mpu6000.c
 *
 *  Created on: Feb 13, 2025
 *      Author: ck
 */

#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/mpu6000.h"

#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define REG_SMPLRT_DIV			0x19

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01

SPI_TypeDef * SPI_MPU6000;

GPIO_TypeDef* GPIO_CS_MPU6000;

uint8_t CS_PIN_MPU6000;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];


}MPU6000_PARAMETERS_t;

MPU6000_PARAMETERS_t mpu6000 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

uint8_t CK_MPU6000_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	for(int i = 0; i < GYRO_READ_ARRAY_SIZE; i++){

		mpu6000.rxArray[i] = 0xFF;
	}

	SPI_MPU6000 = SPIn;
	GPIO_CS_MPU6000 = GPIO_CSn;
	CS_PIN_MPU6000 = CS_PINn;

	uint8_t id = 0;
	CK_SPI_ReadRegisterMulti(MPU_RA_WHO_AM_I, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000, &id, 1);

	if(id == 0x68){
		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_1, 0x80, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMilliSec(100);

		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_1, 0x03, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(10);

		CK_SPI_WriteRegister(MPU_RA_USER_CTRL, 0x10, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

		CK_SPI_WriteRegister(MPU_RA_PWR_MGMT_2, 0x00, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

		CK_SPI_WriteRegister(REG_SMPLRT_DIV, 0, SPI_MPU6000, GPIO_CS_MPU6000, CS_PIN_MPU6000);
		CK_TIME_DelayMicroSec(15);

	}

	return 1;
}

bool CK_MPU6000_isGyroSensorInitialized(void){

	return mpu6000.GyroInit;
}

bool CK_MPU6000_isAccSensorInitialized(void){

    return mpu6000.AccInit;
}





















