
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"
#include "FLASH/CK_FLASH.h"

#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "stdbool.h"

/*
 *
 * General driver class for each flash memory type.
 *
 */

typedef struct{

	uint32_t CURRENT_ADDRESS; // Current address to write data

	uint32_t LAST_ADDRESS;    // The last address can be written to the flash chip

	uint32_t LOG_END_ADDRESS; // The last address written when log ended

	bool is_flash_ready;      // If flash memory erased

	bool is_log_ended;

	uint8_t log_transfer_buffer[512];

	uint8_t paramters_write_buffer[EEPROM_BUFFER_SIZE]; // 128KByte is available

	uint8_t parameters_read_buffer[EEPROM_BUFFER_SIZE];

}flash_memory_t;

flash_memory_t flash = {

	.CURRENT_ADDRESS 	= 1, // First address for log analyzer data (number of address written etc.)
	.LAST_ADDRESS 		= 0,
	.is_flash_ready 	= false, // Is memory erased to write
	.is_log_ended 		= false

};

//flash_type_id_e flash_type_;

flash_type_id_e TARGET_MCU_FLASH;
flash_type_id_e TARGET_LOG_FLASH;

SPI_TypeDef* CK_FLASH_SPI;

GPIO_TypeDef* CK_FLASH_CS_PORT;

uint8_t CK_FLASH_CS_PIN;

void CK_FLASH_Init_Internal(flash_type_id_e type){

	TARGET_MCU_FLASH = type;

	CK_FLASH_INTERNAL_Init();

	// Erase will be used when data is coppied to sector 11
	// else the configured data will be erased and quad used that data to work.

}

void CK_FLASH_Init_External(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, flash_type_id_e type){

	CK_FLASH_SPI = spin_;
	CK_FLASH_CS_PORT = cs_gpio_;
	CK_FLASH_CS_PIN = cs_pin_;

	TARGET_LOG_FLASH = type;

	if(TARGET_LOG_FLASH == FLASH_W25Q128FV_){

		flash.LAST_ADDRESS = 0xFFFFFF;

		CK_FLASH_W25Q128FV_Init(CK_FLASH_SPI, CK_FLASH_CS_PORT, CK_FLASH_CS_PIN);

		CK_FLASH_W25Q128FV_ChipErase();

		flash.is_flash_ready = true;
	}

}

// This method is used to load adjustment data to memory to check if memcpy data offset copy is correct.
// Call this method from main to check.
uint8_t CK_FLASH_WriteTestParameters(flash_type_id_e flash_type){

	if(TARGET_MCU_FLASH == FLASH_STM32H7_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		uint8_t test[EEPROM_BUFFER_SIZE];
		for(int i = 0; i < EEPROM_BUFFER_SIZE; i++){
			test[i] = i;
		}

		if(CK_FLASH_INTERNAL_EraseSector()){

			if(CK_FLASH_Write(test, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH)){

				CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

				// I chenck mechanisim to see if read and writes are correct can be implemented here.
				int a = flash.parameters_read_buffer[15];
				UNUSED(a);

				return 1;

			}

		}

	}

	else if(TARGET_MCU_FLASH == FLASH_STM32F4_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		uint8_t test[EEPROM_BUFFER_SIZE];
		for(int i = 0; i < EEPROM_BUFFER_SIZE; i++){
			test[i] = i;
		}

		if(CK_FLASH_INTERNAL_EraseSector()){

			if(CK_FLASH_Write(test, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH)){

				CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

				// I chenck mechanisim to see if read and writes are correct can be implemented here.
				int a = flash.parameters_read_buffer[15];
				UNUSED(a);

				return 1;

			}

		}

	}

	return 0; // Error

}

uint8_t CK_FLASH_WriteParameters(flash_type_id_e flash_type, uint8_t* buffer, uint16_t buffer_size){

	if(TARGET_MCU_FLASH == FLASH_STM32H7_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		if(CK_FLASH_INTERNAL_EraseSector()){

			if(CK_FLASH_Write(buffer, buffer_size, TARGET_MCU_FLASH)){

				CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

				// I check mechanisim to see if read and writes are correct can be implemented here.
				int a = flash.parameters_read_buffer[15];
				UNUSED(a);

				return 1;

			}

		}

	}

	else if(TARGET_MCU_FLASH == FLASH_STM32F4_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		if(CK_FLASH_INTERNAL_EraseSector()){

			if(CK_FLASH_Write(buffer, buffer_size, TARGET_MCU_FLASH)){

				CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

				// I check mechanisim to see if read and writes are correct can be implemented here.
				int a = flash.parameters_read_buffer[15];
				UNUSED(a);

				return 1;

			}

		}

	}

	return 0; // Error

}

uint8_t CK_FLASH_ReadParameters(flash_type_id_e flash_type, uint8_t* buffer, uint16_t buffer_size, uint16_t offset){

	if(TARGET_MCU_FLASH == FLASH_STM32H7_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		// flash.parameters_read_buffer is memory addres so add offset to it directly
		// or &flash.parameters_read_buffer[offset] is the same
		memcpy(buffer, flash.parameters_read_buffer+offset, buffer_size);
	}

	else if(TARGET_MCU_FLASH == FLASH_STM32F4_){

		CK_FLASH_Read(flash.parameters_read_buffer, EEPROM_BUFFER_SIZE, TARGET_MCU_FLASH);

		// flash.parameters_read_buffer is memory addres so add offset to it directly
		// or &flash.parameters_read_buffer[offset] is the same
		memcpy(buffer, flash.parameters_read_buffer+offset, buffer_size);
	}

	return 0; // Error

}

uint8_t CK_FLASH_Write(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type){

	if(TARGET_MCU_FLASH == FLASH_STM32H7_){

		if(CK_FLASH_INTERNAL_Write(STM32H7_SECTOR7_START_ADDRESS, buffer, (uint16_t)size)){

			return 1; // OK
		}
	}

	else if(TARGET_MCU_FLASH == FLASH_STM32F4_){

		if(CK_FLASH_INTERNAL_Write(STM32F405_SECTOR11_START_ADDRESS, buffer, (uint16_t)size)){

			return 1; // OK
		}
	}

	return 0; // Error
}

void CK_FLASH_Read(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type){

	if(TARGET_MCU_FLASH == FLASH_STM32H7_){

		CK_FLASH_INTERNAL_Read(STM32H7_SECTOR7_START_ADDRESS, buffer, (uint16_t)size);

	}

	else if(TARGET_MCU_FLASH == FLASH_STM32F4_){

		CK_FLASH_INTERNAL_Read(STM32F405_SECTOR11_START_ADDRESS, buffer, (uint16_t)size);

	}

}

uint8_t CK_FLASH_WriteExernal(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type){

	if(TARGET_LOG_FLASH == FLASH_W25Q128FV_){

		// 256 Byte takes 380 microsec without dma
		CK_FLASH_W25Q128FV_WritePage(buffer, size, flash.CURRENT_ADDRESS++);

	}

	return 0; // Error
}

uint8_t CK_FLASH_ReadExernal(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type){

	if(TARGET_LOG_FLASH == FLASH_W25Q128FV_){



	}

	return 0; // Error
}

// CK_LOG will call this update by sending buffer and size
// If dma is implemented it can be update similar to microcard
void CK_FLASH_Update(uint8_t* buffer, uint32_t size){

	if(TARGET_LOG_FLASH == FLASH_W25Q128FV_){

		if(flash.is_flash_ready && !flash.is_log_ended){

			if(flash.CURRENT_ADDRESS < flash.LAST_ADDRESS){

				CK_FLASH_WriteExernal(buffer, size, TARGET_LOG_FLASH);

				flash.LOG_END_ADDRESS = flash.CURRENT_ADDRESS;
			}
			else{

				flash.is_log_ended = true;

				// Add method to write the number of sectors written during log

			}
		}
	}
}

void CK_FLASH_TransferLog(void){

	if(TARGET_LOG_FLASH == FLASH_W25Q128FV_){

		uint32_t num_of_bytes_read = flash.LOG_END_ADDRESS;

		CK_FLASH_W25Q128FV_StartBurstRead(0x000000);

		// Since the address is incremented automatically after sending
		// the first address this method will be used.

		uint16_t index = 0;

		uint8_t data;

		while(num_of_bytes_read--){

			data = CK_SPI_Transfer(CK_FLASH_SPI, 0);

			CK_USBD_WriteTxBuffer(data);
			index++;
			if(index == 512){
				CK_USBD_Transmit();
				index = 0;
			}
		}

		CK_FLASH_W25Q128FV_DeselectChip();
	}

}
