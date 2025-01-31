
#ifndef CK_FLASH_W25Q128FV_H_
#define CK_FLASH_W25Q128FV_H_

#include "CK_DEFINITIONS.h"

void CK_FLASH_W25Q128FV_Init(SPI_TypeDef* spi, GPIO_TypeDef* gpio, uint16_t pin);

void CK_FLASH_W25Q128FV_WritePage(uint8_t* data_buffer, uint16_t write_size, uint32_t start_address);

void CK_FLASH_W25Q128FV_FastRead(uint8_t* readBuffer, uint16_t read_size, uint32_t start_address);

void CK_FLASH_W25Q128FV_Read(uint8_t* readBuffer, uint16_t read_size, uint32_t start_address);

void CK_FLASH_W25Q128FV_StartBurstRead(uint32_t start_address);

void CK_FLASH_W25Q128FV_SectorErase(uint32_t start_address);

void CK_FLASH_W25Q128FV_BlockErase32KB(uint32_t address);

void CK_FLASH_W25Q128FV_BlockErase64KB(uint32_t address);

void CK_FLASH_W25Q128FV_ChipErase(void);

void CK_FLASH_W25Q128FV_WaitBusy(void);

uint8_t CK_FLASH_W25Q128FV_ReadStatusRegister(uint8_t reg);

void CK_FLASH_W25Q128FV_WriteEnable(void);

void CK_FLASH_W25Q128FV_WriteDisable(void);

void CK_FLASH_W25Q128FV_ReadID(void);

uint32_t CK_FLASH_W25Q128FV_GetAddress(uint8_t block_number, uint8_t sector_number, uint8_t page_number, uint8_t byte_of_page);

void CK_FLASH_W25Q128FV_SelectChip(void);

void CK_FLASH_W25Q128FV_DeselectChip(void);


#endif /* CK_FLASH_W25Q128FV_H_ */
