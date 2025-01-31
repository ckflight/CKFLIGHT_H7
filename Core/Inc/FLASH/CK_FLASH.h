
#ifndef FLASH_CK_FLASH_H_
#define FLASH_CK_FLASH_H_

#include "CK_DEFINITIONS.h"

#include "FLASH/CK_FLASH_INTERNAL.h"

#include "FLASH/CK_FLASH_W25Q128FV.h"

#define EEPROM_BUFFER_SIZE	128

typedef enum{

	FLASH_STM32F4_,

	FLASH_STM32H7_,

	FLASH_W25Q128FV_

}flash_type_id_e;

extern flash_type_id_e TARGET_MCU_FLASH;
extern flash_type_id_e TARGET_LOG_FLASH;

void CK_FLASH_Init_Internal(flash_type_id_e type);

void CK_FLASH_Init_External(SPI_TypeDef* spin_, GPIO_TypeDef* cs_gpio_, uint8_t cs_pin_, flash_type_id_e type);

uint8_t CK_FLASH_WriteTestParameters(flash_type_id_e flash_type);

uint8_t CK_FLASH_WriteParameters(flash_type_id_e flash_type, uint8_t* buffer, uint16_t buffer_size);

uint8_t CK_FLASH_ReadParameters(flash_type_id_e flash_type, uint8_t* buffer, uint16_t buffer_size, uint16_t offset);

uint8_t CK_FLASH_Write(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type);

void CK_FLASH_Read(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type);

uint8_t CK_FLASH_WriteExernal(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type);

uint8_t CK_FLASH_ReadExernal(uint8_t* buffer, uint32_t size, flash_type_id_e flash_type);

void CK_FLASH_Update(uint8_t* buffer, uint32_t size);

void CK_FLASH_TransferLog(void);

#endif /* FLASH_CK_FLASH_H_ */
