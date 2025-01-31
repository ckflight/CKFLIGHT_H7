
#include "FLASH/CK_FLASH_INTERNAL.h"

#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_TIME_HAL.h"

//**********************************************************************************
//
// 	Each address location holds 4 byte (32bits) and increased by 4
//  Each row has 16 bytes in total.
//
//  0x080E 0000 to 0x080F FFFF = 0x1FFFF => 131072
//  Reference Manual indicates 128Kbyte but actually it makes
//  131072 bytes = 128 Kib
//
//  If some error happens use ST-Link Utility and performe a full chip erase.
//
//**********************************************************************************
/*
 * IMPORTANT:
 * 1. STLINK is not working (cannot redubug) after flash emulation,
 *    SEGGER handles like a boss use SEGGER.
 * 	  It might be reset pin related since i could not connect reset pin.
 *
 * 2. I updated the linker file and changed the flash memory size and the
 *    last sector is used for flash emulation.
 *
 *    FLASH	 (rx)	: ORIGIN = 0x8000000,	LENGTH = 1024K-128K
 *
 * 3. Nor flash needs to be erase before writing new data since the
 *    a sector is the smallest unit, to write either 1 byte or 128KB
 *    whole sector needed to be erased each time to write data.
 *
 *	  H7 Sector
 *    Bank 1, Sector 7, 0x080E 0000 - 0x080F FFFF
 *    Bank 2, Sector 7, 0x081E 0000 - 0x081F FFFF
 *
 *    F4 Sectors
 *	  ------------------------------------------------------
 *    |0x0800 0000 | 0x0800 0004 | 0x08000 0008 | 0x0800 000C|
 *    ------------------------------------------------------
 *    |0x0800 0010 | 0x0800 0014 | 0x08000 0018 | 0x0800 001C|
 *    ------------------------------------------------------
 *    |0x0800 0020 | 0x0800 0024 | 0x08000 0028 | 0x0800 002C|
 *    ------------------------------------------------------
 */

HAL_StatusTypeDef status;
FLASH_EraseInitTypeDef flash_erase;
uint32_t sector_error = 0;

void CK_FLASH_INTERNAL_Init(void){

#if USE_H7 == 1
	/* Fill EraseInit structure*/
	flash_erase.TypeErase     = FLASH_TYPEERASE_SECTORS;
	flash_erase.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	flash_erase.Banks         = FLASH_BANK_2;
	flash_erase.Sector        = 7;
	flash_erase.NbSectors     = 1;
#endif

#if USE_F4 == 1
	flash_erase.Sector = FLASH_SECTOR_11;
	flash_erase.TypeErase = TYPEERASE_SECTORS;
	flash_erase.NbSectors = 1;
	flash_erase.VoltageRange = VOLTAGE_RANGE_3;
#endif

}

uint8_t CK_FLASH_INTERNAL_Write(uint32_t readStartAddr, uint8_t* write_buffer, uint16_t write_size){

#if USE_H7 == 1

	uint32_t currentAddr = 0;

	uint8_t* writeAddr = 0;

	currentAddr = readStartAddr;

	if(CK_FLASH_INTERNAL_Unlock()){

		uint16_t size = write_size / 32; // flash word is 32 bytes

		writeAddr = write_buffer;

		while (size--){

			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, currentAddr, (uint64_t)(uint32_t)writeAddr) == HAL_OK){

				currentAddr = currentAddr + 32;

				writeAddr = writeAddr + 32;
			}
			else{
				return 0; // Error
			}
		}
	}

	CK_FLASH_INTERNAL_Lock();


	return 1; // OK

#endif

#if USE_F4 == 1

	uint32_t currentAddr = 0;

	currentAddr = readStartAddr;

	if(CK_FLASH_INTERNAL_Unlock()){

		while (write_size--){

			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, currentAddr, *write_buffer++) == HAL_OK){

				currentAddr = currentAddr + 1;
			}
			else{
				return 0; // Error
			}
		}
	}

	CK_FLASH_INTERNAL_Lock();


	return 1; // OK

#endif

}

void CK_FLASH_INTERNAL_Read(uint32_t readStartAddr, uint8_t* read_buffer, uint16_t read_size){

	uint32_t currentAddr = 0;
	uint32_t data32 = 0;

	// Read the sector
	currentAddr = readStartAddr;

	uint32_t size = read_size / 4; // 4 bytes will read at once
	while (size--){

		data32 = *(uint32_t*)currentAddr;

		*read_buffer++ = data32;
		*read_buffer++ = data32 >> 8;
		*read_buffer++ = data32 >> 16;
		*read_buffer++ = data32 >> 24;

		currentAddr = currentAddr + 4;

	}

}

// This one is working correctly
uint8_t CK_FLASH_INTERNAL_EraseSector(void){

#if USE_H7 == 1

	uint8_t resp;

	if(CK_FLASH_INTERNAL_Unlock()){

		//CK_FLASH_INTERNAL_ClearFlags();
		//__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

		//FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

		// I added some time
		//CK_TIME_DelayMilliSec(2000);
		status = FLASH_WaitForLastOperation((uint32_t)5000U, FLASH_BANK_2); // 5 sec timeout

		if(HAL_FLASHEx_Erase(&flash_erase, &sector_error) == HAL_OK){
			resp = 1;
		}

		if(status == HAL_OK){
			resp = 1;
		}
	}

	CK_FLASH_INTERNAL_Lock();

	if(resp){
		return 1; // OK
	}

	return 0; // Error

#endif

#if USE_F4 == 1

	uint8_t resp;

	if(CK_FLASH_INTERNAL_Unlock()){

		CK_FLASH_INTERNAL_ClearFlags();
		//__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

		FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

		// I added some time
		//CK_TIME_DelayMilliSec(2000);
		status = FLASH_WaitForLastOperation((uint32_t)5000U); // 5 sec timeout

		if(status == HAL_OK){
			resp = 1;
		}
	}

	CK_FLASH_INTERNAL_Lock();

	if(resp){
		return 1; // OK
	}

	return 0; // Error

#endif

}

void CK_FLASH_INTERNAL_ClearFlags(void){

#if USE_F4 == 1
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
#endif
}

uint8_t CK_FLASH_INTERNAL_Unlock(void){

	// Unlock the Flash to enable the flash control register access

	if(HAL_FLASH_Unlock() != HAL_OK){
		return 0; // Error
	}
	return 1; // OK

}

void CK_FLASH_INTERNAL_Lock(void){

	 // Lock the Flash to disable flash control

	HAL_FLASH_Lock();

}



