
#include "CK_SPI.h"
#include "CK_TIME_HAL.h"
#include "CK_GPIO.h"

typedef struct{

    uint32_t timeout;

    uint32_t spi1_timeout;
    int spi1_init;
    CK_SPIx_LibraryType spi1_type;

    uint32_t spi2_timeout;
    int spi2_init;
    CK_SPIx_LibraryType spi2_type;

    uint32_t spi3_timeout;
    int spi3_init;
    CK_SPIx_LibraryType spi3_type;

    uint32_t spi4_timeout;
    int spi4_init;
    CK_SPIx_LibraryType spi4_type;

}SPI_t;

SPI_t spi_variables = {

    .timeout      = 0,

    .spi1_timeout = 0,
    .spi1_init    = 0,
	.spi1_type	  = 0,

	.spi2_timeout = 0,
    .spi2_init    = 0,
	.spi2_type	  = 0,

    .spi3_timeout = 0,
    .spi3_init    = 0,
	.spi3_type	  = 0,

	.spi4_timeout = 0,
	.spi4_init    = 0,
	.spi4_type	  = 0,

};

#define SPI_TIMEOUT         100

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

void CK_SPI_Init(SPI_TypeDef* spi_n, uint32_t clock, CK_SPIx_LibraryType type){

	SPI_TypeDef* SPI_ = spi_n;

	if(SPI_ == SPI1){

		__HAL_RCC_SPI1_CLK_ENABLE();

#if USE_SPI1 == true
		CK_GPIO_ClockEnable(SPI1_SCK_GPIO);
		CK_GPIO_ClockEnable(SPI1_MISO_GPIO);
		CK_GPIO_ClockEnable(SPI1_MOSI_GPIO);

		CK_GPIO_Init(SPI1_SCK_GPIO, SPI1_SCK_PIN, CK_GPIO_AF_PP, SPI1_SCK_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI1_MISO_GPIO, SPI1_MISO_PIN, CK_GPIO_AF_PP, SPI1_MISO_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI1_MOSI_GPIO, SPI1_MOSI_PIN, CK_GPIO_AF_PP, SPI1_MOSI_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#endif

	}
	else if(SPI_ == SPI2){

		__HAL_RCC_SPI2_CLK_ENABLE();

#if USE_SPI2 == true
		CK_GPIO_ClockEnable(SPI2_SCK_GPIO);
		CK_GPIO_ClockEnable(SPI2_MISO_GPIO);
		CK_GPIO_ClockEnable(SPI2_MOSI_GPIO);

		CK_GPIO_Init(SPI2_SCK_GPIO, SPI2_SCK_PIN, CK_GPIO_AF_PP, SPI2_SCK_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI2_MISO_GPIO, SPI2_MISO_PIN, CK_GPIO_AF_PP, SPI2_MISO_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI2_MOSI_GPIO, SPI2_MOSI_PIN, CK_GPIO_AF_PP, SPI2_MOSI_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#endif

	}
	else if(SPI_ == SPI3){

		__HAL_RCC_SPI3_CLK_ENABLE();

#if USE_SPI3 == true
		CK_GPIO_ClockEnable(SPI3_SCK_GPIO);
		CK_GPIO_ClockEnable(SPI3_MISO_GPIO);
		CK_GPIO_ClockEnable(SPI3_MOSI_GPIO);

		CK_GPIO_Init(SPI3_SCK_GPIO, SPI3_SCK_PIN, CK_GPIO_AF_PP, SPI3_SCK_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI3_MISO_GPIO, SPI3_MISO_PIN, CK_GPIO_AF_PP, SPI3_MISO_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI3_MOSI_GPIO, SPI3_MOSI_PIN, CK_GPIO_AF_PP, SPI3_MOSI_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#endif

	}
	else if(SPI_ == SPI4){

		__HAL_RCC_SPI4_CLK_ENABLE();

#if USE_SPI4 == true
		CK_GPIO_ClockEnable(SPI4_SCK_GPIO);
		CK_GPIO_ClockEnable(SPI4_MISO_GPIO);
		CK_GPIO_ClockEnable(SPI4_MOSI_GPIO);

		CK_GPIO_Init(SPI4_SCK_GPIO, SPI4_SCK_PIN, CK_GPIO_AF_PP, SPI4_SCK_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI4_MISO_GPIO, SPI4_MISO_PIN, CK_GPIO_AF_PP, SPI4_MISO_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
		CK_GPIO_Init(SPI4_MOSI_GPIO, SPI4_MOSI_PIN, CK_GPIO_AF_PP, SPI4_MOSI_AF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
#endif
	}

	if(type == CK_SPI_USE_HAL){

		SPI_HandleTypeDef hspi_;

		// SPI parameter configuration
		hspi_.Instance 							= SPI_;
		hspi_.Init.Mode 						= SPI_MODE_MASTER;
		hspi_.Init.Direction 					= SPI_DIRECTION_2LINES;
		hspi_.Init.DataSize 					= SPI_DATASIZE_8BIT;
		hspi_.Init.CLKPolarity 					= SPI_POLARITY_LOW;     // SPI_POLARITY_LOW
		hspi_.Init.CLKPhase 					= SPI_PHASE_1EDGE; 		// SPI_PHASE_1EDGE
		hspi_.Init.BaudRatePrescaler 			= clock << 28;
		hspi_.Init.FirstBit 					= SPI_FIRSTBIT_MSB;
		hspi_.Init.TIMode 						= SPI_TIMODE_DISABLE;
		hspi_.Init.CRCCalculation 				= SPI_CRCCALCULATION_DISABLE;
		hspi_.Init.NSS 							= SPI_NSS_SOFT;
		hspi_.Init.NSSPMode 					= SPI_NSS_PULSE_ENABLE;//SPI_NSS_PULSE_DISABLE;
		hspi_.Init.FifoThreshold 				= SPI_FIFO_THRESHOLD_01DATA;
		hspi_.Init.MasterSSIdleness 			= SPI_MASTER_SS_IDLENESS_00CYCLE;
		hspi_.Init.MasterInterDataIdleness 		= SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
		hspi_.Init.MasterReceiverAutoSusp 		= SPI_MASTER_RX_AUTOSUSP_DISABLE;
		hspi_.Init.MasterKeepIOState 			= SPI_MASTER_KEEP_IO_STATE_ENABLE;
		hspi_.Init.IOSwap 						= SPI_IO_SWAP_DISABLE;

		if (HAL_SPI_Init(&hspi_) == HAL_OK){

			if(SPI_ == SPI1){

				__HAL_RCC_SPI1_CLK_ENABLE();

				hspi1 = hspi_;
				spi_variables.spi1_init    	= 1;
				spi_variables.spi1_timeout 	= 0;
				spi_variables.spi1_type 	= CK_SPI_USE_HAL;
			}
			else if(SPI_ == SPI2){

				hspi2 = hspi_;
				spi_variables.spi2_init		= 1;
				spi_variables.spi2_timeout 	= 0;
				spi_variables.spi2_type 	= CK_SPI_USE_HAL;
			}
			else if(SPI_ == SPI3){

				hspi3 = hspi_;
				spi_variables.spi3_init    	= 1;
				spi_variables.spi3_timeout 	= 0;
				spi_variables.spi3_type 	= CK_SPI_USE_HAL;
			}
			else if(SPI_ == SPI4){

				hspi4 = hspi_;
				spi_variables.spi4_init    	= 1;
				spi_variables.spi4_timeout 	= 0;
				spi_variables.spi4_type 	= CK_SPI_USE_HAL;
			}

		}

		// HAL is not configuring correctly.
		SPI_->CR1 = 0;
		SPI_->CR1 |= 1u << 12; // SSI bit

		SPI_->CFG2 = 0;
		SPI_->CFG2 |= 1u << 31 | 1u << 26 | 1u << 22;

		SPI_->CFG1 &= ~(7u << 28);
		SPI_->CFG1 |= clock << 28; // 1 is 16 MHz speed 0 is 32 MHz

	}
	else if(type == CK_SPI_USE_BAREMETAL){

		SPI_->CR1 |= 1u << 12; // SSI bit

		SPI_->CFG1 |= clock << 28 | 7u << 0; // SPI clock, 8 bit data frame,

		// AFCNTR, SSM, clock polarity sck 0 at idle, cpha low, master, spi ti, default msb first, default full duplex
		SPI_->CFG2 |= 1u << 31 | 1u << 26 | 1u << 22;

		if(SPI_ == SPI1){

			spi_variables.spi1_init    	= 1;
			spi_variables.spi1_timeout 	= 0;
			spi_variables.spi1_type 	= CK_SPI_USE_BAREMETAL;
		}
		else if(SPI_ == SPI2){

			spi_variables.spi2_init    	= 1;
			spi_variables.spi2_timeout 	= 0;
			spi_variables.spi2_type 	= CK_SPI_USE_BAREMETAL;
		}
		else if(SPI_ == SPI3){

			spi_variables.spi3_init    	= 1;
			spi_variables.spi3_timeout 	= 0;
			spi_variables.spi3_type 	= CK_SPI_USE_BAREMETAL;
		}
		else if(SPI_ == SPI4){

			spi_variables.spi4_init    	= 1;
			spi_variables.spi4_timeout 	= 0;
			spi_variables.spi4_type 	= CK_SPI_USE_BAREMETAL;
		}
	}

}

void CK_SPI_Enable(SPI_TypeDef* SPI_){

	SPI_->CR1 |= 1u << 0;
}

void CK_SPI_Disable(SPI_TypeDef* SPI_){

	SPI_->CR1 &= ~(1u << 0);
}

void CK_SPI_EnableTXDMA(SPI_TypeDef* SPI_){

	SPI_->CFG1 |= 1u << 15;
}

void CK_SPI_EnableRXDMA(SPI_TypeDef* SPI_){

	SPI_->CFG1 |= 1u << 14;
}

void CK_SPI_DisableTXDMA(SPI_TypeDef* SPI_){

	SPI_->CFG1 &= ~(1u << 15);
}

void CK_SPI_DisableRXDMA(SPI_TypeDef* SPI_){

	SPI_->CFG1 &= ~(1u << 14);
}

void CK_SPI_StartTransfer(SPI_TypeDef* SPI_, uint32_t tsize){

	SPI_->CR2 |= tsize;	 	// tx size is

	SPI_->CR1 |= 1u << 0;   // enable spi

	SPI_->CR1 |= 1u << 9; 	// master tx start

}

void CK_SPI_ChangeClock(SPI_TypeDef* SPI_, CK_SPIx_CR1_Fclk_Div clk){

	SPI_->CFG1 &= ~(7u << 28);
	SPI_->CFG1 |= clk << 28;

}

uint8_t CK_SPI_WriteRegister(uint8_t reg, uint8_t data, SPI_TypeDef* SPI_, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin){

	CK_SPIx_LibraryType spi_type_;
	uint8_t rx_data = 0;

	if(SPI_ == SPI1){
		spi_type_ = spi_variables.spi1_type;
	}
	else if(SPI_ == SPI2){
		spi_type_ = spi_variables.spi2_type;
	}
	else if(SPI_ == SPI3){
		spi_type_ = spi_variables.spi3_type;
	}
	else if(SPI_ == SPI4){
		spi_type_ = spi_variables.spi4_type;
	}

	if(spi_type_ == CK_SPI_USE_HAL){

		SPI_HandleTypeDef hspi_;

		if(SPI_ == SPI1) hspi_ = hspi1;
		else if(SPI_ == SPI2) hspi_ = hspi2;
		else if(SPI_ == SPI3) hspi_ = hspi3;
		else if(SPI_ == SPI4) hspi_ = hspi4;

		CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

		HAL_SPI_Transmit(&hspi_, &reg, 1, 100);

		HAL_SPI_TransmitReceive(&hspi_, &data, &rx_data, 1, 100);

		CK_GPIO_SetPin(GPIOx_CS, cs_pin);
	}
	else if(spi_type_ == CK_SPI_USE_BAREMETAL){

		CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

		CK_SPI_Transfer(SPI_, reg);

		rx_data = CK_SPI_Transfer(SPI_, data);

		CK_GPIO_SetPin(GPIOx_CS, cs_pin);

	}

	return rx_data;

}

void CK_SPI_ReadRegisterMulti(uint8_t reg, SPI_TypeDef* SPI_, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin, uint8_t* dataIn, int count){

	CK_SPIx_LibraryType spi_type_;

	if(SPI_ == SPI1){
		spi_type_ = spi_variables.spi1_type;
	}
	else if(SPI_ == SPI2){
		spi_type_ = spi_variables.spi2_type;
	}
	else if(SPI_ == SPI3){
		spi_type_ = spi_variables.spi3_type;
	}
	else if(SPI_ == SPI4){
		spi_type_ = spi_variables.spi4_type;
	}

	if(spi_type_ == CK_SPI_USE_HAL){

		static uint8_t resp1 = 0;
		static uint8_t resp2 = 0;

		SPI_HandleTypeDef hspi_;

		if(SPI_ == SPI1) hspi_ = hspi1;
		else if(SPI_ == SPI2) hspi_ = hspi2;
		else if(SPI_ == SPI3) hspi_ = hspi3;
		else if(SPI_ == SPI4) hspi_ = hspi4;

		uint8_t read_reg = reg|0x80;

		CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

		resp1 = HAL_SPI_Transmit(&hspi_, &read_reg, 1, 10);

		resp2 = HAL_SPI_Receive(&hspi_, dataIn, count, 10);

		CK_GPIO_SetPin(GPIOx_CS, cs_pin);

		if(resp1 == HAL_TIMEOUT || resp2 == HAL_TIMEOUT){
			CK_SPI_TimeOutCounter((&hspi_)->Instance);
		}
	}
	else if(spi_type_ == CK_SPI_USE_BAREMETAL){

		CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

		CK_SPI_Transfer(SPI_, reg | 0x80);

		while (count--) {

			*dataIn++ =  CK_SPI_Transfer(SPI_, 0xFF);
		}

		CK_GPIO_SetPin(GPIOx_CS, cs_pin);

		/*
		CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

		uint8_t is_register_sent = 0;
		uint8_t c = count + 1;

		CK_SPI_StartTransfer(SPI_, c);

		while(c){

			// TXP Flag
			spi_variables.timeout = SPI_TIMEOUT;
			while((SPI_->SR & (1u << 1)) == 0){
				if(--spi_variables.timeout == 0){
					CK_SPI_TimeOutCounter(SPI_);
					break;
				}
			}

			if(is_register_sent == 0){
				*((__IO uint8_t *)&SPI_->TXDR) = reg | 0x80;
			}
			else{
				*((__IO uint8_t *)&SPI_->TXDR) = 0xFF;
			}

			// RXP Flag
			spi_variables.timeout = SPI_TIMEOUT;
			while((SPI_->SR & (1u << 0)) == 0){
				if(--spi_variables.timeout == 0){
					CK_SPI_TimeOutCounter(SPI_);
					break;

				}
			}

			if(is_register_sent == 0){
				is_register_sent = 1;
				uint8_t b = *((__IO uint8_t *)&SPI_->RXDR);
				UNUSED(b);
			}
			else{
				*dataIn++ = *((__IO uint8_t *)&SPI_->RXDR);
			}

			--c;

		}

	    // EOT Flag
		spi_variables.timeout = SPI_TIMEOUT;
		while((SPI_->SR & (1u << 3)) == 0){
			if(--spi_variables.timeout == 0){
				CK_SPI_TimeOutCounter(SPI_);
				break;
			}
		}

		SPI_->IFCR |= 1u << 4; // Clear txtf

		SPI_->CR1 &= ~(1u << 0); // disable spi

		CK_GPIO_SetPin(GPIOx_CS, cs_pin);

		*/

	}

}

uint8_t CK_SPI_Transfer(SPI_TypeDef* SPI_, uint8_t data){

	CK_SPIx_LibraryType spi_type_;
	uint8_t rx_data = 0;

	if(SPI_ == SPI1){
		spi_type_ = spi_variables.spi1_type;
	}
	else if(SPI_ == SPI2){
		spi_type_ = spi_variables.spi2_type;
	}
	else if(SPI_ == SPI3){
		spi_type_ = spi_variables.spi3_type;
	}
	else if(SPI_ == SPI4){
		spi_type_ = spi_variables.spi4_type;
	}

	if(spi_type_ == CK_SPI_USE_HAL){

		SPI_HandleTypeDef hspi_;

		if(SPI_ == SPI1) hspi_ = hspi1;
		else if(SPI_ == SPI2) hspi_ = hspi2;
		else if(SPI_ == SPI3) hspi_ = hspi3;
		else if(SPI_ == SPI4) hspi_ = hspi4;

		static uint8_t resp = 0;

		resp = HAL_SPI_Transmit(&hspi_, &data, 1, 100);

		if(resp == HAL_TIMEOUT){
			CK_SPI_TimeOutCounter((&hspi_)->Instance);
		}

	}
	else if(spi_type_ == CK_SPI_USE_BAREMETAL){

		CK_SPI_StartTransfer(SPI_, 1);

		// TXP Flag
		spi_variables.timeout = SPI_TIMEOUT;
		while((SPI_->SR & (1u << 1)) == 0){
			if(--spi_variables.timeout == 0x00){
				CK_SPI_TimeOutCounter(SPI_);
				break;
			}
		}

		*((__IO uint8_t *)&SPI_->TXDR) = data;

		// RXP Flag
		spi_variables.timeout = SPI_TIMEOUT;
		while((SPI_->SR & (1u << 0)) == 0){
			if(--spi_variables.timeout == 0x00){
				CK_SPI_TimeOutCounter(SPI_);
				break;
			}
		}

		rx_data = *((__IO uint8_t *)&SPI_->RXDR);

		// EOT Flag
		spi_variables.timeout = SPI_TIMEOUT;
		while((SPI_->SR & (1u << 3)) == 0){
			if(--spi_variables.timeout == 0){
				CK_SPI_TimeOutCounter(SPI_);
				break;
			}
		}

		SPI_->IFCR |= 1u << 4;		 	// Clear txtf
		SPI_->CR1 &= ~(1u << 0); 		// disable spi

	}

	return rx_data;

}

uint8_t CK_SPI_WaitTransfer(SPI_TypeDef* SPI_){

	spi_variables.timeout = SPI_TIMEOUT;

	//while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
	while((SPI_->SR & (1u << 1)) == 0){
		if(--spi_variables.timeout == 0x00){
			CK_SPI_TimeOutCounter(SPI_);
			return 1;
		}
	}

	return 0;
}

int CK_SPI_CheckInitialized(SPI_TypeDef* SPI_){

    int res;
    if(SPI_ == SPI1){
        res = spi_variables.spi1_init;
    }
    else if(SPI_ == SPI2){
        res = spi_variables.spi2_init;
    }
    else if(SPI_ == SPI3){
        res = spi_variables.spi3_init;
    }
    else if(SPI_ == SPI4){
		res = spi_variables.spi4_init;
	}
    else{
        res = 2; // Error
    }

	return res;

}

void CK_SPI_TimeOutCounter(SPI_TypeDef* SPI_){

	if(SPI_ == SPI1){
        spi_variables.spi1_timeout++;
    }
    else if(SPI_ == SPI2){
        spi_variables.spi2_timeout++;
    }
    else if(SPI_ == SPI3){
        spi_variables.spi3_timeout++;
    }
    else if(SPI_ == SPI4){
		spi_variables.spi4_timeout++;
	}
}

uint32_t CK_SPI_GetTimeOut(SPI_TypeDef* SPI_){

    uint32_t res;

    if(SPI_ == SPI1){
        res = spi_variables.spi1_timeout;
    }
    else if(SPI_ == SPI2){
        res = spi_variables.spi2_timeout;
    }
    else if(SPI_ == SPI3){
        res = spi_variables.spi3_timeout;
    }
    else if(SPI_ == SPI4){
		res = spi_variables.spi4_timeout;
	}

    return res;
}

void CK_SPI_ResetTimeOut(SPI_TypeDef* SPI_){

    if(SPI_ == SPI1){
        spi_variables.spi1_timeout = 0;
    }
    else if(SPI_ == SPI2){
        spi_variables.spi2_timeout = 0;
    }
    else if(SPI_ == SPI3){
        spi_variables.spi3_timeout = 0;
    }
    else if(SPI_ == SPI4){
		spi_variables.spi4_timeout = 0;
	}

}

CK_SPIx_CR1_Fclk_Div CK_SPI_GetClockRate(SPI_TypeDef* SPI_, uint32_t spi_clock){

	uint16_t clock_rate = 0;

	if(SPI_ == SPI1){
		clock_rate = 64000000 / spi_clock;
	}
	else if(SPI_ == SPI2){
		clock_rate = 64000000 / spi_clock;
	}
	else if(SPI_ == SPI3){
		clock_rate = 64000000 / spi_clock;
	}
	else if(SPI_ == SPI4){
		clock_rate = 120000000 / spi_clock;
	}
	else if(SPI_ == SPI5){
		clock_rate = 120000000 / spi_clock;
	}

	CK_SPIx_CR1_Fclk_Div clock = 0;

	if(clock_rate <= 2){
		clock = CK_SPIx_CR1_Fclk_Div2;
	}
	else if(clock_rate > 2 && clock_rate <= 4){
		clock = CK_SPIx_CR1_Fclk_Div4;
	}
	else if(clock_rate > 4 && clock_rate <= 8){
		clock = CK_SPIx_CR1_Fclk_Div8;
	}
	else if(clock_rate > 8 && clock_rate <= 16){
		clock = CK_SPIx_CR1_Fclk_Div16;
	}
	else if(clock_rate > 16 && clock_rate <= 32){
		clock = CK_SPIx_CR1_Fclk_Div32;
	}
	else if(clock_rate > 32 && clock_rate <= 64){
		clock = CK_SPIx_CR1_Fclk_Div64;
	}
	else if(clock_rate > 64 && clock_rate <= 128){
		clock = CK_SPIx_CR1_Fclk_Div128;
	}
	else if(clock_rate > 128 && clock_rate <= 256){
		clock = CK_SPIx_CR1_Fclk_Div256;
	}

	return clock;
}

