
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_L3GD20H.h"

#include "MOTION/CK_GYRO.h"


#define CK_L3GD20H_CTRL1                    0x20
#define CK_L3GD20H_CTRL2                    0x21
#define CK_L3GD20H_CTRL4                    0x23
#define CK_L3GD20H_CTRL5                    0x24
#define CK_L3GD20H_WHO_AM_I_ID              0xD7
#define CK_L3GD20_WHO_AM_I_ID               0xD4

#define CK_L3GD20H_WHO_AM_I_READ_REG        0x0F|0x80 // If we set MSB, indicates a read operation

#define CK_L3GD20H_OUT_X_L                  0x28
#define CK_L3GD20H_OUT_X_H                  0x29

#define CK_L3GD20H_OUT_Y_L                  0x2A
#define CK_L3GD20H_OUT_Y_H                  0x2B

#define CK_L3GD20H_OUT_Z_L                  0x2C
#define CK_L3GD20H_OUT_Z_H                  0x2D

SPI_TypeDef * SPI_L3GD20H;

GPIO_TypeDef* GPIO_CS_L3GD20H;

uint8_t CS_PIN_L3GD20H;

typedef struct{

    bool GyroInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];

}L3GD20H_PARAMETERS_t;

L3GD20H_PARAMETERS_t l3dg20h = {
    .GyroInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

uint8_t CK_L3GD20H_Init(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	SPI_L3GD20H = SPIn;
	GPIO_CS_L3GD20H = GPIO_CSn;
	CS_PIN_L3GD20H = CS_PINn;

	uint8_t resp = 0;

	if(CK_SPI_WriteRegister(CK_L3GD20H_WHO_AM_I_READ_REG, 0xFF, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H) == CK_L3GD20H_WHO_AM_I_ID){

		//L3GD20H Setup
		CK_SPI_WriteRegister(CK_L3GD20H_CTRL1, 0xFF, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H);//ODR: 800Hz, BW: 100HZ //800,760 Difference between l3gd20 and l3gd20h

		CK_SPI_WriteRegister(CK_L3GD20H_CTRL4, 0x20, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H);//DPS Selection 2000deg/sec

		CK_SPI_WriteRegister(CK_L3GD20H_CTRL2, 0x00, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H);//HP Filter Configuration

		CK_SPI_WriteRegister(CK_L3GD20H_CTRL5, 0x10, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H);//HP Filter Enable

		l3dg20h.GyroInit = true;

		l3dg20h.HardwareInit = true;

		resp = 1;
	}
	else{
		resp = 0;
	}

	return resp;

}

void CK_L3GD20H_AlignGyro(int x, int y, int z){

	gyro.gyroSign[X]  = x;

	gyro.gyroSign[Y] = y;

	gyro.gyroSign[Z]   = z;

}

void CK_L3GD20H_ReadGyroRaw(void){

    CK_SPI_ReadRegisterMulti(CK_L3GD20H_OUT_X_L, SPI_L3GD20H, GPIO_CS_L3GD20H, CS_PIN_L3GD20H, l3dg20h.rxArray, 6);

    gyro.gyroADCRaw[X] = (int16_t)(l3dg20h.rxArray[0] << 8 | l3dg20h.rxArray[1]);

    gyro.gyroADCRaw[Y] = (int16_t)(l3dg20h.rxArray[2] << 8 | l3dg20h.rxArray[3]);

    gyro.gyroADCRaw[Z] = (int16_t)(l3dg20h.rxArray[4] << 8 | l3dg20h.rxArray[5]);


}

bool CK_L3GD20H_isGyroSensorInitialized(void){

	return l3dg20h.GyroInit;
}


