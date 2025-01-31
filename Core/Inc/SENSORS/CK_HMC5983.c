
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/CK_HMC5983.h"

#include "MOTION/CK_MAGNETO.h"

#define HMC5983_ADDRESS         0x1E

#define HMC5983_CTRL_REG_A      0x00
#define HMC5983_CTRL_REG_B      0x01
#define HMC5983_CTRL_REG_C      0x02

#define HMC5983_XOUT_MSB        0x03

#define READ_ARRAY_SIZE         6

I2C_TypeDef* HMC5983_I2C;

typedef struct{

    bool MagInit;

    uint8_t rxArray[READ_ARRAY_SIZE];


}HMC5983_PARAMETERS_t;

HMC5983_PARAMETERS_t hmc5983 = {
    .MagInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

void CK_HMC5983_Init(I2C_TypeDef* I2Cn, uint32_t magFreq){

	HMC5983_I2C = I2Cn;

	// Later put who am i check

	// Register Setup before Active Mode
	CK_I2C_Transfer(HMC5983_I2C, HMC5983_ADDRESS, HMC5983_CTRL_REG_A, 0x1C); // 220Hz normal measurement no sample averaging
	CK_I2C_Transfer(HMC5983_I2C, HMC5983_ADDRESS, HMC5983_CTRL_REG_B, 0x20); // +- 1.3Gauss
	CK_I2C_Transfer(HMC5983_I2C, HMC5983_ADDRESS, HMC5983_CTRL_REG_C, 0x00); // Continuous conversion

    mag.magScale[0] = 0.00092f; //Digital resolution is 0.00092 Gauss/LSB or 1090 LSB/Gauss

    hmc5983.MagInit = true;

}

void CK_HMC5983_AlignMag(int x, int y, int z){

	mag.magSign[X] = x;

	mag.magSign[Y] = y;

	mag.magSign[Z] = z;

}

void CK_HMC5983_ReadMagRaw(void){

	CK_I2C_ReadMulti(HMC5983_I2C, HMC5983_ADDRESS, HMC5983_XOUT_MSB, hmc5983.rxArray, 6);

	// Its register order is X, Z, Y
	mag.magADCRaw[X] = (int16_t)(hmc5983.rxArray[0] << 8 | hmc5983.rxArray[1]);

	mag.magADCRaw[Z] = (int16_t)(hmc5983.rxArray[2] << 8 | hmc5983.rxArray[3]);

	mag.magADCRaw[Y] = (int16_t)(hmc5983.rxArray[4] << 8 | hmc5983.rxArray[5]);

}

bool CK_HMC5983_isMagSensorInitialized(void){

    return hmc5983.MagInit;
}
