
#include "CK_I2C.h"
#include "CK_SYSTEM.h"
#include "CK_GPIO.h"

#define I2C_TIMEOUT                         500// 500 makes around 50 usec which is enough.

CK_I2C_LIB lib_mode;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

typedef struct{

    uint32_t timeout;

    uint32_t i2c1_timeout;
    int i2c1_init;

    uint32_t i2c2_timeout;
    int i2c2_init;

    uint32_t i2c3_timeout;
    int i2c3_init;

    uint32_t i2c4_timeout;
    int i2c4_init;

}I2C_t;

I2C_t i2c_variables = {

    .timeout      = 0,

    .i2c1_timeout = 0,
    .i2c1_init    = 0,

    .i2c2_timeout = 0,
    .i2c2_init    = 0,

    .i2c3_timeout = 0,
    .i2c3_init    = 0,

    .i2c4_timeout = 0,
    .i2c4_init    = 0,
};

void CK_I2C_Init(I2C_TypeDef* i2c, CK_I2C_Speed freq, CK_I2C_LIB lib){

	lib_mode = lib;

	if(lib_mode  == USE_HAL_I2C){

		I2C_HandleTypeDef hi2c_;

		if(i2c == I2C1){

			// Peripheral clock enable
			__HAL_RCC_I2C1_CLK_ENABLE();

			// I2C1 GPIO Configuration
			// PB6     ------> I2C1_SCL
			// PB7     ------> I2C1_SDA

			__HAL_RCC_GPIOB_CLK_ENABLE();
			CK_GPIO_Init(GPIOB, 6, CK_GPIO_AF_OD, CK_GPIO_AF4, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
			CK_GPIO_Init(GPIOB, 7, CK_GPIO_AF_OD, CK_GPIO_AF4, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

		}

		else if(i2c == I2C2){

			// Peripheral clock enable
			__HAL_RCC_I2C2_CLK_ENABLE();

			// I2C2 GPIO Configuration
			// PB10     ------> I2C1_SCL
			// PB11     ------> I2C1_SDA
			__HAL_RCC_GPIOB_CLK_ENABLE();
			CK_GPIO_Init(GPIOB, 10, CK_GPIO_AF_OD, CK_GPIO_AF4, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
			CK_GPIO_Init(GPIOB, 11, CK_GPIO_AF_OD, CK_GPIO_AF4, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

		}

		int freq_i2c = 0;
		if(freq == CK_I2C_100Khz){
			freq_i2c = 120;
		}
		else if(freq == CK_I2C_400Khz){
			freq_i2c = 400; // 420 makes close to 400 KHz
		}
		uint32_t i2cPclk 	= CK_SYSTEM_GetAPB1Clock();
		uint32_t i2c_timing = CK_I2C_ClockTIMINGR(i2cPclk, freq_i2c, 0);

		hi2c_.Instance 				= i2c;
		hi2c_.Init.Timing 			= i2c_timing;
		hi2c_.Init.OwnAddress1 		= 0;
		hi2c_.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT;
		hi2c_.Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
		hi2c_.Init.OwnAddress2 		= 0;
		hi2c_.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		hi2c_.Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
		hi2c_.Init.NoStretchMode 	= I2C_NOSTRETCH_DISABLE;

		if (HAL_I2C_Init(&hi2c_) == HAL_OK){

			// Configure Analogue filter
			if (HAL_I2CEx_ConfigAnalogFilter(&hi2c_, I2C_ANALOGFILTER_ENABLE) == HAL_OK){

				// Configure Digital filter
				if (HAL_I2CEx_ConfigDigitalFilter(&hi2c_, 0) == HAL_OK){

					if(i2c == I2C1){
						hi2c1 = hi2c_;
						i2c_variables.i2c1_init = 1;
						i2c_variables.i2c1_timeout = 0;
					}
					else if(i2c == I2C2){
						hi2c2 = hi2c_;
						i2c_variables.i2c2_init = 1;
						i2c_variables.i2c2_timeout = 0;
					}
					else if(i2c == I2C3){
						hi2c3 = hi2c_;
						i2c_variables.i2c3_init = 1;
						i2c_variables.i2c3_timeout = 0;
					}

				}

			}

		}

	}
	else{



	}

}

uint16_t CK_I2C_IsBusy(I2C_TypeDef* I2Cx){
	return 0;
}

void CK_I2C_Transfer(I2C_TypeDef* i2c_, uint8_t slaveAddress, uint8_t reg, uint8_t data){

	if(lib_mode == USE_HAL_I2C){

		I2C_HandleTypeDef i2c_handler;

		if(i2c_ == I2C1){

			i2c_handler = hi2c1;
		}
		else if(i2c_ == I2C2){

			i2c_handler = hi2c2;
		}
		else if(i2c_ == I2C3){

			i2c_handler = hi2c3;
		}
		else if(i2c_ == I2C4){

			i2c_handler = hi2c4;
		}

		HAL_I2C_Mem_Write(&i2c_handler, slaveAddress << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 5); // Timeout is in millisec

	}
	else{

	}

}

void CK_I2C_ReadMultiInterrupt(I2C_HandleTypeDef* i2c_handler, uint8_t slaveAddress, uint8_t reg, uint8_t* rxBuffer, int quantity){

}

void CK_I2C_ReadMulti(I2C_TypeDef* i2c_, uint8_t slaveAddress, uint8_t reg, uint8_t* rxBuffer, int quantity){

	if(lib_mode == USE_HAL_I2C){

		I2C_HandleTypeDef i2c_handler;

		if(i2c_ == I2C1){

			i2c_handler = hi2c1;
		}
		else if(i2c_ == I2C2){

			i2c_handler = hi2c2;
		}
		else if(i2c_ == I2C3){

			i2c_handler = hi2c3;
		}
		else if(i2c_ == I2C4){

			i2c_handler = hi2c4;
		}

		HAL_I2C_Mem_Read(&i2c_handler, slaveAddress << 1, reg, I2C_MEMADD_SIZE_8BIT, rxBuffer, quantity, 5); // Timeout is in millisec

	}
	else{


	}

}

void CK_I2C_Start(I2C_TypeDef* I2Cx, uint8_t slaveAddress, CK_I2C_Mode mode, CK_I2C_ACK_Mode ack){


}

void CK_I2C_Stop(I2C_TypeDef* I2Cx){

}

uint8_t CK_I2C_ReadAck(I2C_TypeDef* I2Cx){
	return 0;
}

uint8_t CK_I2C_ReadNack(I2C_TypeDef* I2Cx){
	return 0;
}

ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT){
	return SUCCESS;
}

int CK_I2C_CheckInitialized(I2C_TypeDef* I2Cn){

    int res;
    if(I2Cn == I2C1){
        res = i2c_variables.i2c1_init;
    }
    else if(I2Cn == I2C2){
        res = i2c_variables.i2c2_init;
    }
    else if(I2Cn == I2C3){
        res = i2c_variables.i2c3_init;
    }
    else if(I2Cn == I2C4){
		res = i2c_variables.i2c4_init;
	}
    else{
        res = 2; // Error
    }

    return res;

}

void CK_I2C_TimeOutCounter(I2C_TypeDef* I2Cn){

	if(I2Cn == I2C1){
        i2c_variables.i2c1_timeout++;
    }
    else if(I2Cn == I2C2){
        i2c_variables.i2c2_timeout++;
    }
    else if(I2Cn == I2C3){
        i2c_variables.i2c3_timeout++;
    }
    else if(I2Cn == I2C4){
		i2c_variables.i2c4_timeout++;
	}
}

uint32_t CK_I2C_GetTimeOut(I2C_TypeDef* I2Cn){

    uint32_t res;

    if(I2Cn == I2C1){
        res = i2c_variables.i2c1_timeout;
    }
    else if(I2Cn == I2C2){
        res = i2c_variables.i2c2_timeout;
    }
    else if(I2Cn == I2C3){
        res = i2c_variables.i2c3_timeout;
    }
    else if(I2Cn == I2C4){
		res = i2c_variables.i2c4_timeout;
	}

    return res;
}

void CK_I2C_ResetTimeOut(I2C_TypeDef* I2Cn){

    if(I2Cn == I2C1){
        i2c_variables.i2c1_timeout = 0;
    }
    else if(I2Cn == I2C2){
        i2c_variables.i2c2_timeout = 0;
    }
    else if(I2Cn == I2C3){
        i2c_variables.i2c3_timeout = 0;
    }
    else if(I2Cn == I2C4){
		i2c_variables.i2c4_timeout = 0;
	}

}

// Compute SCLDEL, SDADEL, SCLH and SCLL for TIMINGR register according to reference manuals.
void CK_I2C_ClockComputeRaw(uint32_t pclkFreq, int i2cFreqKhz, int presc, int dfcoeff, uint8_t *scldel, uint8_t *sdadel, uint16_t *sclh, uint16_t *scll){

	// Values from I2C-SMBus specification
    uint16_t trmax;      // Rise time (max)
    uint16_t tfmax;      // Fall time (max)
    uint8_t tsuDATmin;   // SDA setup time (min)
    uint8_t thdDATmin;   // SDA hold time (min)
    uint16_t tHIGHmin;   // High period of SCL clock (min)
    uint16_t tLOWmin;    // Low period of SCL clock (min)

    // Silicon specific values, from datasheet
    uint8_t tAFmin = 50; // Analog filter delay (min)

    // Actual (estimated) values
    uint8_t tr = 100;   // Rise time
    uint8_t tf = 10;    // Fall time

    if (i2cFreqKhz > 400) {
        // Fm+ (Fast mode plus)
        trmax = 120;
        tfmax = 120;
        tsuDATmin = 50;
        thdDATmin = 0;
        tHIGHmin = 260;
        tLOWmin = 500;
    } else {
        // Fm (Fast mode)
        trmax = 300;
        tfmax = 300;
        tsuDATmin = 100;
        thdDATmin = 0;
        tHIGHmin = 600;
        tLOWmin = 1300;
    }

    // Convert pclkFreq into nsec
    float tI2cclk = 1000000000.0f / pclkFreq;

    // Convert target i2cFreq into cycle time (nsec)
    float tSCL = 1000000.0f / i2cFreqKhz;

    uint32_t SCLDELmin = (trmax + tsuDATmin) / ((presc + 1) * tI2cclk) - 1;
    uint32_t SDADELmin = (tfmax + thdDATmin - tAFmin - ((dfcoeff + 3) * tI2cclk)) / ((presc + 1) * tI2cclk);

    float tsync1 = tf + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;
    float tsync2 = tr + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;

    float tSCLH = tHIGHmin * tSCL / (tHIGHmin + tLOWmin) - tsync2;
    float tSCLL = tSCL - tSCLH - tsync1 - tsync2;

    uint32_t SCLH = tSCLH / ((presc + 1) * tI2cclk) - 1;
    uint32_t SCLL = tSCLL / ((presc + 1) * tI2cclk) - 1;

    while (tsync1 + tsync2 + ((SCLH + 1) + (SCLL + 1)) * ((presc + 1) * tI2cclk) < tSCL) {
        SCLH++;
    }

    *scldel = SCLDELmin;
    *sdadel = SDADELmin;
    *sclh = SCLH;
    *scll = SCLL;
}

uint32_t CK_I2C_ClockTIMINGR(uint32_t pclkFreq, int i2cFreqKhz, int dfcoeff){

#define TIMINGR(presc, scldel, sdadel, sclh, scll) \
    ((presc << 28)|(scldel << 20)|(sdadel << 16)|(sclh << 8)|(scll << 0))

    uint8_t scldel;
    uint8_t sdadel;
    uint16_t sclh;
    uint16_t scll;

    for (int presc = 0; presc < 15; presc++) {
    	CK_I2C_ClockComputeRaw(pclkFreq, i2cFreqKhz, presc, dfcoeff, &scldel, &sdadel, &sclh, &scll);

        // If all fields are not overflowing, return TIMINGR.
        // Otherwise, increase prescaler and try again.
        if ((scldel < 16) && (sdadel < 16) && (sclh < 256) && (scll < 256)) {
            return TIMINGR(presc, scldel, sdadel, sclh, scll);
        }
    }

    return 0; // Shouldn't reach here

}

