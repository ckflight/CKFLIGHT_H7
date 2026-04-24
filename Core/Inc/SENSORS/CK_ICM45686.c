
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_ICM45686.h"


SPI_TypeDef * SPI_ICM45686;

GPIO_TypeDef* GPIO_CS_ICM45686;

uint8_t CS_PIN_ICM45686;

typedef struct{

    bool GyroInit;

    bool AccInit;

    bool HardwareInit;

    uint8_t rxArray[GYRO_READ_ARRAY_SIZE];


}ICM20602_PARAMETERS_t;

ICM20602_PARAMETERS_t icm45686 = {
    .GyroInit = false,
    .AccInit = false,
    .HardwareInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

uint8_t CK_ICM45686_GyroInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t gyroFreq){

	for(int i = 0; i < GYRO_READ_ARRAY_SIZE; i++){

		icm45686.rxArray[i] = 0xFF;
	}

	SPI_ICM45686 = SPIn;
	GPIO_CS_ICM45686 = GPIO_CSn;
	CS_PIN_ICM45686 = CS_PINn;

	uint8_t resp = 0;

	uint8_t id = CK_SPI_WriteRegister(0x72|0x80, 0xFF, SPI_ICM45686, GPIO_CS_ICM45686, CS_PIN_ICM45686);
	if(id == 0xE9){


		resp = 1;

	}
	else{

	    resp = 0;
	}

	return resp;

}

uint8_t CK_ICM45686_AccInit(SPI_TypeDef* SPIn, GPIO_TypeDef* GPIO_CSn, uint8_t CS_PINn, uint32_t accFreq){

    SPI_ICM45686 = SPIn;
    GPIO_CS_ICM45686 = GPIO_CSn;
    CS_PIN_ICM45686 = CS_PINn;

    uint8_t resp = 0;

    if(CK_SPI_WriteRegister(0x72, 0xFF, SPI_ICM45686, GPIO_CS_ICM45686, CS_PIN_ICM45686) == 0xE9){

    	int a = 0;
    	a++;

    }
    else{
    	resp = 0;
    }

    return resp;

}

bool CK_ICM45686_isGyroSensorInitialized(void){

	return icm45686.GyroInit;
}
