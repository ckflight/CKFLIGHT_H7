
#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_BUZZER.h"

#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_ACC.h"

#include "SENSORS/CK_FXOS8700CQ.h"

#define FXOS8700_ADDRESS                    0x1F

I2C_TypeDef* FXOS8700CQ_I2C;

typedef struct{

    bool AccInit;

    bool MagInit;

    uint8_t rxArray[ACC_READ_ARRAY_SIZE];


}FXOS8700CQ_PARAMETERS_t;

FXOS8700CQ_PARAMETERS_t fxos8700cq = {
    .AccInit = false,
    .MagInit = false,
    .rxArray = {0, 0, 0, 0, 0, 0}
};

void CK_FXOS8700CQ_AccInit(I2C_TypeDef* I2Cn, uint32_t accFreq){

    FXOS8700CQ_I2C = I2Cn;

	// FXOS8700
	//ACC
	CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x2A, 0x00); // CTRL_REG1, STANDBY
	CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x0E, 0x01); // XYZ_DATA_CFG, +-4g 0.488 mg/LSB

	CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x2B, 0x02); // CTRL_REG2, High Resolution Mode
	CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x2A, 0x15); // CTRL_REG1, Hybrid 100Hz, Low Noise, Active

	fxos8700cq.AccInit = true;

}

void CK_FXOS8700CQ_MagInit(I2C_TypeDef* I2Cn, uint32_t magFreq){

    FXOS8700CQ_I2C = I2Cn;

    //MAG
    CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x5B, 0x1F); // M_CTRL_REG1, Oversampling 16 100Hz hybrind, acc mag active
    CK_I2C_Transfer(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x5C, 0x20); // M_CTRL_REG2, Jump to reg 0x33 after reading 0x06

    mag.magScale[0] = 0.10f;

    fxos8700cq.MagInit = true;

}

void CK_FXOS8700CQ_AlignAcc(int x, int y, int z){

	acc.accSign[X]  = x;

	acc.accSign[Y] = y;

	acc.accSign[Z]   = z;

}

void CK_FXOS8700CQ_AlignMag(int x, int y, int z){

	mag.magSign[X]  = x;

	mag.magSign[Y] = y;

	mag.magSign[Z]   = z;

}

void CK_FXOS8700CQ_ReadAccRaw(void){

	CK_I2C_ReadMulti(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x01, fxos8700cq.rxArray, 6);

	uint8_t axhi = fxos8700cq.rxArray[0];
	uint8_t axlo = fxos8700cq.rxArray[1];
	uint8_t ayhi = fxos8700cq.rxArray[2];
	uint8_t aylo = fxos8700cq.rxArray[3];
	uint8_t azhi = fxos8700cq.rxArray[4];
	uint8_t azlo = fxos8700cq.rxArray[5];

	acc.accADCRaw[X] = (int16_t)((axhi << 8) | axlo) >> 2;

	acc.accADCRaw[Y] = (int16_t)((ayhi << 8) | aylo) >> 2;

	acc.accADCRaw[Z] = (int16_t)((azhi << 8) | azlo) >> 2;

}

void CK_FXOS8700CQ_ReadMagRaw(void){

	// M_OUT_X_MSB 0x33
	CK_I2C_ReadMulti(FXOS8700CQ_I2C, FXOS8700_ADDRESS, 0x33, fxos8700cq.rxArray, 6);

	uint8_t mxhi = fxos8700cq.rxArray[0];
	uint8_t mxlo = fxos8700cq.rxArray[1];
	uint8_t myhi = fxos8700cq.rxArray[2];
	uint8_t mylo = fxos8700cq.rxArray[3];
	uint8_t mzhi = fxos8700cq.rxArray[4];
	uint8_t mzlo = fxos8700cq.rxArray[5];

	mag.magADCRaw[X] = (int16_t)((mxhi << 8) | mxlo);

	mag.magADCRaw[Y] = (int16_t)((myhi << 8) | mylo);

	mag.magADCRaw[Z] = (int16_t)((mzhi << 8) | mzlo);

}


bool CK_FXOS8700CQ_isAccSensorInitialized(void){

    return fxos8700cq.AccInit;
}

bool CK_FXOS8700CQ_isMagSensorInitialized(void){

    return fxos8700cq.MagInit;
}




