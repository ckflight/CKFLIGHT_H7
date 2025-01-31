
#ifndef DRIVERS_CK_FLASH_H_
#define DRIVERS_CK_FLASH_H_

#include "CK_DEFINITIONS.h"

#define STM32H7_SECTOR7_START_ADDRESS	(uint32_t)0x081E0000
#define STM32H7_SECTOR7_END_ADDRESS		(uint32_t)0x081FFFFF

#define STM32F405_SECTOR11_START_ADDRESS	(uint32_t)0x080E0000
#define STM32F405_SECTOR11_END_ADDRESS		(uint32_t)0x080FFFFF


void CK_FLASH_INTERNAL_Init(void);

uint8_t CK_FLASH_INTERNAL_Write(uint32_t readStartAddr, uint8_t* write_buffer, uint16_t write_size);

void CK_FLASH_INTERNAL_Read(uint32_t readStartAddr, uint8_t* read_buffer, uint16_t read_size);

uint8_t CK_FLASH_INTERNAL_EraseSector(void);

void CK_FLASH_INTERNAL_ClearFlags(void);

uint8_t CK_FLASH_INTERNAL_Unlock(void);

void CK_FLASH_INTERNAL_Lock(void);

#endif
