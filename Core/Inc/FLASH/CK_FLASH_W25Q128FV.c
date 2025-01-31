
#include "FLASH/CK_FLASH_W25Q128FV.h"

#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_TIME_HAL.h"

//**********************************************************************************
//
//						  WINBOND Flash Memory Map
//  W25Q128FV 128MBit
//	Entire Chip = 256 Blocks (each 64KB)
//	Each Block  = 16 Sectors (each 4KB)
//  Each Sector = 16 Pages   (each 256Bytes)
//
//  ADDRESSING: 0x(00)0000 upper most 2 byte of 6 byte address changes with block
//                         00 is first block to FF is last 255th block
//
//				0x00(00)00 middle 2 bytes of 6 byte address changes with sectors in block
//						   00 to 0F is first sector, 10 to 1F second ... F0 to FF is last
//
//				0x0000(00) last 2 bytes of 6 byte address changes with page bytes in sector
//						   00 to FF is 256 byte page
//
// 			    Example: 0x000000 first byte of first block's first sectors' page
// 			    Example: 0x000010 16th byte of first block's first sectors' page
//
// 			    Example: 0x000100 first byte of first block's second sectors' page
// 			    Example: 0x0001FF 256th byte of first block's second sectors' page
//
//
// Erasing options:
// 1. 4KB Sector Erase: Pages can be erased in groups of 16
// 2. 32KB Block Erase: Pages can be erased in groups of 128
// 3. 64KB Block Erase: Pages can be erased in groups of 256
// 4. Full Chip Erase
//

#define READ_DEVICE_ID_REG		0x9F
#define STATUS_REGISTER_1		0x05
#define STATUS_REGISTER_2		0x35
#define STATUS_REGISTER_3		0x15

#define WRITE_ENABLE_REG		0x06
#define WRITE_DISABLE_REG		0x04

#define SECTOR_ERASE_REG		0x20
#define BLOCK_ERASE32KB_REG		0x52
#define BLOCK_ERASE64KB_REG		0xD8
#define CHIP_ERASE_REG			0xC7 // or 0x60

#define PAGE_PROGRAM_REG		0x02

#define READ_DATA_REG			0x03
#define FAST_READ_REG			0x0B

SPI_TypeDef* FLASH_EXTERNAL_SPI;
GPIO_TypeDef* FLASH_EXTERNAL_GPIO;
uint16_t FLASH_EXTERNAL_CS_PIN;

void CK_FLASH_W25Q128FV_Init(SPI_TypeDef* spi, GPIO_TypeDef* gpio, uint16_t pin){

	FLASH_EXTERNAL_SPI = spi;
	FLASH_EXTERNAL_GPIO = gpio;
	FLASH_EXTERNAL_CS_PIN = pin;

	CK_FLASH_W25Q128FV_DeselectChip();

	CK_FLASH_W25Q128FV_ReadID();

}

void CK_FLASH_W25Q128FV_WritePage(uint8_t* data_buffer, uint16_t write_size, uint32_t start_address){

	/*
	 * 1 to 256 Bytes can be written
	 * The area to write needs to be erased before any write
	 *
	 * If the entire page (256 Bytes) will be programmed the
	 * last byte of the address should be set to 0
	 *
	 * After sector erase check busy of chip
	 */

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_WriteEnable();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, PAGE_PROGRAM_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 16);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 8);
	if(write_size == 256){
		CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);
	}
	else{
		CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address);
	}

	uint16_t size = write_size;
	uint8_t data;
	while(size--){

		data = *data_buffer++;
		CK_SPI_Transfer(FLASH_EXTERNAL_SPI, data);
	}


	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_FastRead(uint8_t* readBuffer, uint16_t read_size, uint32_t start_address){

	// Fast read is similart to read, but it can operate at possibly high clock freq.
	// Add one dummy byte to the end of address write to start fast write.

	// Adress will be auto shifted to next during reading
	// entire chip can be read continuously

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, FAST_READ_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 16);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 8);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address);

	// Dummy byte to start fast read
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);

	uint16_t size = read_size;
	while(size--){

		*readBuffer++ = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);
	}

	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_Read(uint8_t* readBuffer, uint16_t read_size, uint32_t start_address){

	// Adress will be auto shifted to next during reading
	// entire chip can be read continuously

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, READ_DATA_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 16);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 8);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address);

	uint16_t size = read_size;
	while(size--){

		*readBuffer++ = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);
	}

	CK_FLASH_W25Q128FV_DeselectChip();

}


void CK_FLASH_W25Q128FV_StartBurstRead(uint32_t start_address){

	// Adress will be auto shifted to next during reading
	// entire chip can be read continuously

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, FAST_READ_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 16);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 8);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address);

	// Dummy byte to start fast read
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);

}

void CK_FLASH_W25Q128FV_SectorErase(uint32_t start_address){

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_WriteEnable();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, SECTOR_ERASE_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 16);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address >> 8);
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, start_address);

	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_BlockErase32KB(uint32_t address){

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_WriteEnable();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, BLOCK_ERASE32KB_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address >> 16));
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address >> 8));
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address));

	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_BlockErase64KB(uint32_t address){

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_WriteEnable();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, BLOCK_ERASE64KB_REG);

	// Send memory address
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address >> 16));
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address >> 8));
	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, (uint8_t)(address));

	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_ChipErase(void){

	CK_FLASH_W25Q128FV_WaitBusy();

	CK_FLASH_W25Q128FV_WriteEnable();

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, CHIP_ERASE_REG);

	CK_FLASH_W25Q128FV_DeselectChip();

	CK_FLASH_W25Q128FV_WaitBusy();

	// After busy, busy flag and write enable is cleared

}

void CK_FLASH_W25Q128FV_WaitBusy(void){

	uint8_t status;
	do{
		status = CK_FLASH_W25Q128FV_ReadStatusRegister(STATUS_REGISTER_1);

	}while((status & 0x01));

}


uint8_t CK_FLASH_W25Q128FV_ReadStatusRegister(uint8_t reg){

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, reg);

	uint8_t stat = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0);

	CK_FLASH_W25Q128FV_DeselectChip();

	return stat;
}

void CK_FLASH_W25Q128FV_WriteEnable(void){
	/*
	 * Sets the write enable latch bit in the status register.
	 *
	 * The WEL bit must be set prior to every Page Program,
	 * Quad Page Program, Sector Erase, Block Erase, Chip Erase,
	 * Write Status Register and Erase/Program Security Registers instruction
	 */

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, WRITE_ENABLE_REG);

	CK_FLASH_W25Q128FV_DeselectChip();

}

void CK_FLASH_W25Q128FV_WriteDisable(void){
	/*
	 * Resets the write enable latch bit in the status register.
	 *
	 * the WEL bit is automatically reset after Power-up and upon completion of the
	 * Write Status Register, Erase/Program Security Registers, Page Program,
	 * Quad Page Program, Sector Erase, Block Erase, Chip Erase and Reset instructions.
	 */

	CK_FLASH_W25Q128FV_SelectChip();

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, WRITE_DISABLE_REG);

	CK_FLASH_W25Q128FV_DeselectChip();

}


void CK_FLASH_W25Q128FV_ReadID(void){

	CK_FLASH_W25Q128FV_SelectChip();

	uint8_t id_results[3];
	UNUSED(id_results);

	CK_SPI_Transfer(FLASH_EXTERNAL_SPI, READ_DEVICE_ID_REG);

	id_results[0] = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0x00); // Winbond Serial Flash id = 0xEF
	id_results[1] = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0x00); // SPI mode 0x40, QPI mode 0x60
	id_results[2] = CK_SPI_Transfer(FLASH_EXTERNAL_SPI, 0x00); // 0x18

	CK_FLASH_W25Q128FV_DeselectChip();

}

/*
 * Block   		from 0 to 255
 * Sectors 		from 0 to 15
 * Pages   		from 0 to 15
 * Page bytes	from 0 to 255
 */
uint32_t CK_FLASH_W25Q128FV_GetAddress(uint8_t block_number, uint8_t sector_number, uint8_t page_number, uint8_t byte_of_page){

	uint32_t address;

	uint8_t sec_hex = sector_number << 4;

	sec_hex |= page_number;

	address = (uint32_t)((block_number << 16) | (sec_hex << 8) | (byte_of_page));

	return address;

}

void CK_FLASH_W25Q128FV_SelectChip(void){

	CK_GPIO_ClearPin(FLASH_EXTERNAL_GPIO, FLASH_EXTERNAL_CS_PIN);
}

void CK_FLASH_W25Q128FV_DeselectChip(void){

	CK_GPIO_SetPin(FLASH_EXTERNAL_GPIO, FLASH_EXTERNAL_CS_PIN);
}






