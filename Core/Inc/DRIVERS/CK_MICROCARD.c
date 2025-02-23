
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"
#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/CK_PRINTER.h"

#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_SPI.h"
#include "DRIVERS/CK_SPI_DMA.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_LED.h"
#include "DRIVERS/CK_MICROCARD.h"

#include "FLASH/CK_FLASH.h"

#include "FLIGHT/CK_LOG.h"

#define CMD0        0
#define CMD6        6
#define CMD8        8
#define CMD9        9
#define CMD12       12
#define CMD17       17
#define CMD18       18
#define CMD24       24
#define CMD25       25
#define CMD55       55
#define CMD58       58
#define ACMD41      41

#define R1_RESPONSE_READY                	0x00 // Bit 0
#define R1_RESPONSE_IDLE            		0x01 // Bit 0
#define R1_RESPONSE_ILLEGAL_CMD         	0x04 // Bit 1

#define MICROCARD_1        1
#define MICROCARD_2        2

// Root directory attributes
#define DIR_ATTR_READ_ONLY        	0x01
#define DIR_ATTR_HIDDEN            	0x02
#define DIR_ATTR_SYSTEM            	0x04
#define DIR_ATTR_VOLUME_ID        	0x08
#define DIR_ATTR_DIRECTORY        	0x10
#define DIR_ATTR_ARCHIVE        	0x20

#define SD_TIMEOUT             		((uint32_t)0x00100000U)

// 50MHz max clock speed
// (not working yet but needed as well)
//#define HIGH_SPEED_MODE

//#define DEBUG_WRITE_TIMING

microcard_parameters_t card = {
    .init_retry                 	= 50,

    .is_card_fast               	= false,   // Is microcard support fast mode.
    .is_Initialized             	= false,   // Is microcard initialized.
    .is_dma_ready               	= true,    // DMA transfer is ready to start again.
    .is_card_high_capacity      	= false,   // Is microcard high capacity.
	.is_multi_started				= false,

    .card_version                 	= 0,
    .card_speed_clock             	= 0,

    // Start sector to start logging
    .START_SECTOR                	= 0,

    // Current sector the data is logged.
    .CURRENT_SECTOR               	= 0,

    // Each card has an offset. For example to write to sector n, n+offset needed to be sent.
    .SECTOR_OFFSET                	= 0,

    // This sector will be used to write how many sector are written and any later info can be wrtitten.
    // Python code needs to know how much to read.
    .INFO_SECTOR                	= 0,

    .max_number_of_sector_write 	= 0,
    .is_logging_done            	= false,   // When all sectors are written this flag is used.
    .is_infoSector_write        	= false,   // When finally the first sector is filled with logging info this flag is used.
    .TIME_OUT                   	= 100      // some part uses * 10 time_out so 100msec is fine

};

typedef struct{

    uint16_t BPB_BytsPerSec;	// Byte size of each sector (Offset 11, 2 bytes)
    uint8_t  BPB_SecPerCluster;	// Number of sector of a cluster (Offset 13, 1 byte)
    uint16_t BPB_RsvdSecCnt;	// Number of reserved sectors (Offset 14, 2 bytes)
    uint8_t  BPB_NumFATs;		// Number of FATs (Offset 16, 1 byte)
    uint32_t BPB_TotSec32;		// Total number of sectors (Offset 32, 4 bytes)
    uint32_t BPB_FATSz32;		// Number of sectors used by FAT (Offset 36, 4 bytes)
    uint32_t BPB_RootClus;		// This is set to the cluster number of the first cluster of the root directory, this value should be 2 (Offset 44, 4 bytes)

    uint32_t firstRootDirectorSector;

}microcard_bpb_t;

// Each created file is listed in root directory sector(firstRootDirectorSector)
// with 32 bytes of information for each file
// find the name of the file and decode its 32byte information to fill below variables.
// Later i can create file at desired size as well.

typedef struct{

    uint8_t  DIR_Name[11];		// Name of the DRIVE (Offset 0, 11 bytes)
    uint8_t  DIR_Attr;          // Directory attributes (Offset 11, 1 byte)
    uint16_t DIR_LstAccDate;    // Last access date (Offset 18, 2 bytes)
    uint16_t DIR_FstClusHI;     // High bytes of first cluster. This where the log is started (Offset 20, 2 bytes
    uint16_t DIR_FstClusLo;     // High bytes of first cluster. This where the log is started (Offset 26, 2 bytes
    uint32_t DIR_FileSize;      // File size (Offset 28, 4 bytes)

    uint32_t fileFirstCluster;

    uint32_t firsSectorOfFile;

}microcard_rootdirectory_t;

microcard_bpb_t boot_sector;

microcard_rootdirectory_t log_file;

#if defined(DEBUG_WRITE_TIMING)
uint32_t sector_update_time;
uint32_t sector_start_time;
uint32_t sector_results[5000]; // make size equal to .multi_number_of_sector
int sectorIndex = 0;

uint32_t busy_update_time;
uint32_t busy_start_time;
uint32_t busy_results[5000]; // make size equal to .multi_number_of_sector
int busyIndex = 0;
#endif

microcard_finish_modes_e finish_mode;

DEBUG_TIME_t micro_debug;

bool isSingleInfoSectorFinished = false;

SPI_TypeDef* CK_MICROCARD_SPI;

GPIO_TypeDef* CK_MICROCARD_CS_PORT;

uint8_t CK_MICROCARD_CS_PIN;

DMA_TypeDef* CK_MICROCARD_DMA;

DMA_Stream_TypeDef* CK_MICROCARD_DMA_STREAM;

SD_HandleTypeDef hsd1;
/*
 * It takes 400 microsecond for one sector 512 bytes 10MHz
 * It takes 200 microsecond for one sector 512 bytes 20MHz
 *
 * Check clock line with oscilloscope each dma buffer takes 400 microsecond and there must be
 * 2 byte transfer before each dma.
 *
 * 8 KHz sends 2 dma i could not find why.
 *
 * Microcard needs spi clock so i provided spi dummy between dma
 * data logging is working.
 *
 * Tested with 1 Gbyte log 30 minutes 128 bytes logging. It works.
 */

void CK_MICROCARD_Init(microcard_transfer_modes_e mode){

	/*
	 *  SPI version is not checked on H7 system. DMA will be different.
	 *  Now first i will implement SDIO 4B
	 *
	 *  I changed SDMMC_CMDTIMEOUT to 200ms from 5 second in stm32h7xx_II_sdmmc.h line 311
	 */

    card.transfer_mode = mode;

    card.intial_transfer_mode = mode;  // multi mode changes mode to single when writing info so initial mode must be known

    finish_mode = IDLE;

#if LOG_SDIO_

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
	PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	__HAL_RCC_SDMMC1_CLK_ENABLE();

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/**SDMMC1 GPIO Configuration
	PD2    ------> SDMMC1_CMD
	PC11   ------> SDMMC1_D3
	PC10   ------> SDMMC1_D2
	PC12   ------> SDMMC1_CK
	PC9    ------> SDMMC1_D1
	PC8    ------> SDMMC1_D0
	*/
	GPIO_InitStruct.Pin 		= GPIO_PIN_2;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_SDIO1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 		= GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_8;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_SDIO1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

	hsd1.Instance 					= MICROCARD_SDIO;
	hsd1.Init.ClockEdge 			= SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockPowerSave 		= SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide 				= SDMMC_BUS_WIDE_4B;
	hsd1.Init.HardwareFlowControl 	= SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv 				= 1; // Clock = 64 MHz / (ClockDiv + 1)

	// This function starts hardware, inits card with low speed 1B mode
	// gets card status and reconfigures high speed and 4B mode
	if(HAL_SD_Init(&hsd1) == HAL_OK){

		card.is_Initialized = true;

		// Set these two by reading the data from hal library later.

		card.is_card_high_capacity = true;

		card.is_card_fast = true;

	}

#endif

#if LOG_SPI_

	CK_MICROCARD_SPI 		= MICROCARD_SPI;

	CK_MICROCARD_CS_PORT 	= MICROCARD_CS_PORT;

	CK_MICROCARD_CS_PIN 	= MICROCARD_CS_PIN;

	CK_MICROCARD_DMA 		= MICROCARD_DMA;

	CK_MICROCARD_DMA_STREAM = MICROCARD_DMA_STREAM;

    finish_mode = IDLE;

    CK_MICROCARD_DeselectCard();

    uint8_t init_counter = 3; // try max 3 times to init card.

    while(!card.is_Initialized && init_counter--){

    	CK_MICROCARD_InitCard();

    	CK_TIME_DelayMilliSec(250);
    }

#endif

    if(card.is_Initialized){

		#if LOG_SPI_

		CK_MICROCARD_SPIFullSpeed();

		// Configure SPI_DMA

		CK_SPI_DMA_EnableClock(MICROCARD_DMA);

		CK_SPI_DMA_ClearFlag(MICROCARD_DMA, MICROCARD_DMA_STREAM);

#if USE_H7 == 1
		CK_SPI_DMA_InitTX(MICROCARD_DMA_STREAM, MICROCARD_SPI, MICROCARD_DMA_Request1);

		CK_SPI_DMA_SetPeripheralAddress(MICROCARD_DMA_STREAM, (uint32_t)(&MICROCARD_SPI->TXDR));

#endif

#if USE_F4 == 1
		CK_SPI_DMA_InitTX(MICROCARD_DMA_STREAM, MICROCARD_DMA_Channel);

		CK_SPI_DMA_SetPeripheralAddress(MICROCARD_DMA_STREAM, (uint32_t)(&MICROCARD_SPI->DR));

#endif

		CK_SPI_DMA_TCInterruptEnable(MICROCARD_DMA_STREAM);

		HAL_NVIC_EnableIRQ(MICROCARD_DMA_TX_IRQn);

		#endif

		CK_MICROCARD_AccessCardDetails();

		CK_BUZZER_Tone1();

		CK_LED_EnableLed(2);
	}

}

void CK_MICROCARD_InitCard(void){

    uint8_t ocr_register[4];
    uint8_t csd_register[16];
    uint8_t highspeed_response_register[64]; UNUSED(highspeed_response_register);
    uint8_t resp = 0xFF;

    CK_TIME_DelayMilliSec(100);

    while(resp != R1_RESPONSE_IDLE && card.init_retry--){
        CK_MICROCARD_SelectCard();
        resp = CK_MICROCARD_SendCmd(CMD0, 0, 0x95);
        CK_MICROCARD_DeselectCard();
    }

    if (resp == R1_RESPONSE_IDLE){

        CK_MICROCARD_SelectCard();

        // CMD8 is for initialization of version 2.0 compatible card.
        // CMD8 response is R3 = R1 + 4 byte OCR
        resp = CK_MICROCARD_SendCmd(CMD8, 0x000001AB, 0x95);

        // Read OCR register
        if(resp == R1_RESPONSE_IDLE){
            for(int i = 0; i < 4 ; i++){
                ocr_register[i] = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
            }
            if(ocr_register[3] == 0xAB){
                card.card_version = MICROCARD_2; // Version 2
            }
        }
        CK_MICROCARD_DeselectCard();

        if(card.card_version  == MICROCARD_2){

            // It takes a few attempts to get 0 response

            resp = 0xFF;
            CK_TIME_SetTimeOut(card.TIME_OUT);
            while(resp != R1_RESPONSE_READY && CK_TIME_GetTimeOut()){
                CK_MICROCARD_SelectCard();
                resp = CK_MICROCARD_SendAppCommand(ACMD41, 1 << 30);
                CK_MICROCARD_DeselectCard();
            }

            // The Card is initialized
            if(resp == R1_RESPONSE_READY){
                CK_MICROCARD_SelectCard();
                resp = CK_MICROCARD_SendCmd(CMD58, 0, 0x95);// Read OCR Register

                for(int i = 0; i < 4 ; i++){
                    ocr_register[i] = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                }

                CK_MICROCARD_DeselectCard();

                uint32_t ocr = (ocr_register[0] << 24) | (ocr_register[1] << 16) | (ocr_register[2] << 8) | (ocr_register[0]);
                if(resp == R1_RESPONSE_READY){

                    card.is_card_high_capacity = ((ocr & (1<<30)) != 0);

                    card.is_card_fast = true;
                }


#if defined(HIGH_SPEED_MODE)
                /*
                 * HIGH SPEED MODE
                 * For cards v1.10 SDHC CL10 and higher supports high speed mode
                 * The maximum spi clock will be 50MHz rather than 25MHz
                 *
                 */
                CK_TIME_SetTimeOut(card.TIME_OUT);
                CK_MICROCARD_SelectCard();

                uint8_t resp = 1;
                while(resp != 0 && CK_TIME_GetTimeOut()){
                    resp = CK_MICROCARD_SendCmd(CMD6, 0x80000001, 0);
                }

                // 512 bits (64 bytes) result will be readed.
                for(int i = 0; i < 64; i++){
                    highspeed_response_register[i] = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                }

                // At least 8 clock cycle is needed.
                CK_SPI_Transfer(MICROCARD_SPI, 0xFF);
                CK_SPI_Transfer(MICROCARD_SPI, 0xFF);

                CK_MICROCARD_DeselectCard();
#endif

                /* MicroCard Specifications.pdf in my Design Notes
                 *
                 * CSD register gives every information about the card
                 *
                 * In Normal    Mode TRAN_SPEED(4th byte) is 0x32 which is 25MHz max freq.
                 * In HighSpeed Mode TRAN_SPEED(4th byte) is 0x5A which is 50MHz max freq.
                 *
                 * Switch Function command (CMD6), the Version 1.10 and higher memory card
                 * can be placed in High-Speed mode.
                 *
                 * */

                CK_MICROCARD_SelectCard();

                resp = CK_MICROCARD_SendCmd(CMD9, 0, 0);

                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFE);

                for(int byte = 0; byte < 16 ; byte++){
                    csd_register[byte] = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                    if(byte == 2){
                        if(csd_register[byte] == 0x32){
                            card.card_speed_clock = 25;
                        }
                        else if(csd_register[byte] == 0x5A){
                            card.card_speed_clock = 50;
                        }
                    }
                }
                CK_MICROCARD_DeselectCard();

                card.is_Initialized = true;

            }

        }
    }
    else{
        card.is_Initialized = false;

        CK_GPIO_SetPin(CK_MICROCARD_CS_PORT, CK_MICROCARD_CS_PIN);
    }

}

// Each card has an offset. When sector 0 is wanted to be read or written
// offset + sector number must be sent.
// FAT32 has specific chars at sector 0. This function looks for that chars until it finds.
// The number it has found that chars is the offset.
bool CK_MICROCARD_ReadOffset(void){

    // BPB sector 0 (this 0 is actually 0 + offset) so the offset must be found to operate correctly.
    // BPB first byte is 0xEB so read until reaching that byte to determine the offset of the card.

	uint8_t buffer[512];
    uint32_t sector_offset = 0;
    uint32_t sector_max = 100000;

    bool is_bpb_found = false;

    while(sector_offset < sector_max){

        CK_MICROCARD_ReadData(sector_offset, 1, buffer);

        if(buffer[0] != 0xEB && buffer[2] != 0x90){

            sector_offset += 512;

            // Could not find and read BPB sector
            if(sector_offset == sector_max){

            	card.SECTOR_OFFSET = 0;
            	is_bpb_found = false;
            }

        }
        else{
			card.SECTOR_OFFSET = sector_offset;
			is_bpb_found = true;

			break; // Finish while loop
        }

    }

    return is_bpb_found;

}

void CK_MICROCARD_SaveOffset(uint32_t sector_offset){

	uint8_t buffer[512];

	// Read the content of flash first
	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, buffer, EEPROM_BUFFER_SIZE, CONFIG_ID_OFFSET);

	buffer[CONFIG_SD_OFFSET] 	 = 'C';
	buffer[CONFIG_SD_OFFSET + 1] = 'K';

	buffer[CONFIG_SD_OFFSET + 2] = (sector_offset >> 24) & 0xFF;
	buffer[CONFIG_SD_OFFSET + 3] = (sector_offset >> 16) & 0xFF;
	buffer[CONFIG_SD_OFFSET + 4] = (sector_offset >> 8) & 0xFF;
	buffer[CONFIG_SD_OFFSET + 5] = sector_offset & 0xFF;

	// Save offset to the flash's configuration location
	CK_FLASH_WriteParameters(TARGET_MCU_FLASH, buffer, EEPROM_BUFFER_SIZE);
}


void CK_MICROCARD_AccessCardDetails(void){

    // All details are in Microsoft File Allocation Table Specification.pdf
    // First sector has bpb information and from there,
    // number of the first sector of the root file is calculated.

	uint8_t buffer[512];
	bool is_bpb_found = false;

	CK_FLASH_ReadParameters(TARGET_MCU_FLASH, buffer, CONFIG_SD_BYTES, CONFIG_SD_OFFSET);

	// Indication bytes to see if it is configured before.
	// But if user has changed the sd card then the offset might not be the same for other card.
	// Therefore, read one sector to check if it is correct for the card.
	if(buffer[0] == 'C' && buffer[1] == 'K'){

		card.SECTOR_OFFSET  = buffer[2] << 24 | buffer[3] << 16 | buffer[4] << 8 | buffer[5];

		// Check if sector_offset is correct and read bpb sector
		CK_MICROCARD_ReadData(0, 1, buffer); // Read bpb sector 0 + offset, and only 1 sector

		if(buffer[0] == 0xEB && buffer[2] == 0x90){

			// Offset is correct and finds the Bpb sector
			is_bpb_found = true;

		}
		else{

			card.SECTOR_OFFSET = 0;

			// Find the offset for the new card
			is_bpb_found = CK_MICROCARD_ReadOffset();

			if(is_bpb_found){
				CK_MICROCARD_SaveOffset(card.SECTOR_OFFSET);
			}

		}

	}
	else{

		card.SECTOR_OFFSET = 0;

		// Find the offset for the new card
		is_bpb_found = CK_MICROCARD_ReadOffset();

		if(is_bpb_found){
			CK_MICROCARD_SaveOffset(card.SECTOR_OFFSET);
		}

	}

    if(is_bpb_found){

        // Read the boot sector, this function adds offset
        CK_MICROCARD_ReadData(0, 1, buffer); // Read bpb sector 0 + offset, and only 1 sector

        /*
         uint16_t BPB_BytsPerSec;     	// Byte size of each sector (Offset 11 2 bytes)
         uint8_t  BPB_SecPerCluster;	// Number of sector of a cluster (Offset 13 1 byte)
         uint16_t BPB_RsvdSecCnt;     	// Number of reserved sectors (Offset 14 2 bytes)
         uint8_t  BPB_NumFATs;        	// Number of FATs (Offset 16 1 byte)
         uint32_t BPB_TotSec32;        	// Total number of sectors (Offset 32 4 bytes)
         uint32_t BPB_FATSz32;        	// Number of sectors used by FAT (Offset 36 4 bytes)
         */

        // Boot sector bytes are in little endian format meaning LSB first
        boot_sector.BPB_BytsPerSec      = (uint16_t)(buffer[12] << 8 | buffer[11]);

        boot_sector.BPB_SecPerCluster 	= (uint8_t)buffer[13];

        boot_sector.BPB_RsvdSecCnt    	= (uint16_t)(buffer[15] << 8 | buffer[14]);

        boot_sector.BPB_NumFATs         = (uint8_t)buffer[16];

        boot_sector.BPB_TotSec32      	= (uint32_t)(buffer[35] << 24 | buffer[34] << 16 | buffer[33] << 8 | buffer[32]);

        boot_sector.BPB_FATSz32			= (uint32_t)(buffer[39] << 24 | buffer[38] << 16 | buffer[37] << 8 | buffer[36]);

        boot_sector.BPB_RootClus      	= (uint32_t)(buffer[47] << 24 | buffer[46] << 16 | buffer[45] << 8 | buffer[44]);

        boot_sector.firstRootDirectorSector = boot_sector.BPB_RsvdSecCnt + (boot_sector.BPB_FATSz32 * boot_sector.BPB_NumFATs);

        // Read the first root directory sector where info of files are stored.
        CK_MICROCARD_ReadData(boot_sector.firstRootDirectorSector, 1, buffer); // Read sector firstRootDirectorSector, and only 1 sector

        /*
         uint8_t  DIR_Name[11];		// Name of the DRIVE (Offset 0, 11 bytes)
         uint8_t  DIR_Attr;			// Directory attributes (Offset 11, 1 byte)
         uint16_t DIR_LstAccDate;   // Last access date (Offset 18, 2 bytes)
         uint16_t DIR_FstClusHI;    // High bytes of first cluster. This where the log is started (Offset 20, 2 bytes
         uint16_t DIR_FstClusLo;    // High bytes of first cluster. This where the log is started (Offset 26, 2 bytes
         uint32_t DIR_FileSize;     // File size (Offset 28, 4 bytes)
         */

        // Even if the name of file is flight_log.txt this is how it is stored
        uint8_t filename_to_look[11] = {'F','L','I','G','H','T','~','1','T','X','T'};
        uint16_t start_byte_of_file = CK_MICROCARD_GetStartByteOfFile(buffer, filename_to_look);

        for(int i = 0; i < 11; i++){
            log_file.DIR_Name[i] = buffer[start_byte_of_file + i];
        }

        log_file.DIR_Attr = buffer[start_byte_of_file + 11];

        log_file.DIR_LstAccDate = (uint16_t)(buffer[start_byte_of_file + 19] << 8 | buffer[start_byte_of_file + 18]);

        log_file.DIR_FstClusHI = (uint16_t)(buffer[start_byte_of_file + 21] << 8 | buffer[start_byte_of_file + 20]);

        log_file.DIR_FstClusLo = (uint16_t)(buffer[start_byte_of_file + 27] << 8 | buffer[start_byte_of_file + 26]);

        log_file.DIR_FileSize = (uint32_t)(buffer[start_byte_of_file + 31] << 24 | buffer[start_byte_of_file + 30] << 16 | buffer[start_byte_of_file + 29] << 8 | buffer[start_byte_of_file + 28]);

        log_file.fileFirstCluster = (uint32_t)(log_file.DIR_FstClusHI << 16 | log_file.DIR_FstClusLo);

        log_file.firsSectorOfFile = ((log_file.fileFirstCluster - boot_sector.BPB_RootClus) * boot_sector.BPB_SecPerCluster) + boot_sector.firstRootDirectorSector;

        card.START_SECTOR = log_file.firsSectorOfFile + card.SECTOR_OFFSET;

        CK_LED_ToggleLedForMs(2, 50, 20); // Fast toggle
    }
    else{

    	// If could not find and read then i will set some default parameters

        boot_sector.BPB_BytsPerSec      	= 512;
        boot_sector.BPB_SecPerCluster 		= 16;
        boot_sector.BPB_RsvdSecCnt    		= 32;
        boot_sector.BPB_NumFATs         	= 2;
        boot_sector.BPB_TotSec32      		= 30539776;
        boot_sector.BPB_FATSz32				= 14898;
        boot_sector.BPB_RootClus      		= 2;
        boot_sector.firstRootDirectorSector = 29828;

        log_file.DIR_Attr 					= 32;
        log_file.DIR_LstAccDate 			= 21870;
        log_file.DIR_FstClusHI 				= 0;
        log_file.DIR_FstClusLo 				= 164;
        log_file.DIR_FileSize 				= 104857600;
        log_file.fileFirstCluster 			= 164;
        log_file.firsSectorOfFile 			= 40000;

        card.SECTOR_OFFSET = 0;

        card.START_SECTOR = log_file.firsSectorOfFile;

        CK_LED_ToggleLedForMs(2, 20, 50); // Slow toggle
    }

    // First sector will be used to store data needed to be known by python log-analyzer code.
    // I moved it to second sector.
    card.INFO_SECTOR = card.START_SECTOR;

    // Next sector will be the start of logging.
    card.START_SECTOR++;

    // Based on the file size calculate maximum number of sector can be written.
    card.max_number_of_sector_write = log_file.DIR_FileSize / boot_sector.BPB_BytsPerSec;

}

void CK_MICROCARD_WriteInfoSector(void){

	if(card.transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){
	    CK_SPI_DMA_SetBuffer(CK_MICROCARD_DMA_STREAM, flightLog.info_buffer, INFO_BUFFER_SIZE);

	    CK_MICROCARD_WriteData(card.INFO_SECTOR);
	}
	else if(card.transfer_mode == SPI_DMA_INTERRUPT_SINGLEBLOCK){
	    CK_SPI_DMA_SetBuffer(CK_MICROCARD_DMA_STREAM, flightLog.info_buffer, INFO_BUFFER_SIZE);

	    CK_MICROCARD_WriteData(card.INFO_SECTOR);
	}
	else if(card.transfer_mode == SDIO_DMA_INTERRUPT_MULTIBLOCK){

		#if USE_H7 == 1
		// Clean before tx operation when dcache is enabled
		// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
		SCB_CleanDCache_by_Addr((uint32_t*)flightLog.info_buffer, INFO_BUFFER_SIZE + 32);
		#endif

		HAL_SD_WriteBlocks_DMA(&hsd1, flightLog.info_buffer, card.INFO_SECTOR, 1);
	}

}

void CK_MICROCARD_WriteData(uint32_t sector){

    uint8_t resp; UNUSED(resp);
    int num;

    switch(card.transfer_mode){

        case SPI_DMA_INTERRUPT_MULTIBLOCK:

        	if(!card.is_multi_started){
				// SPI will not be deselected until all sectors are written
				CK_MICROCARD_SelectCard();

				// This is also needed to be send once
				resp = CK_MICROCARD_SendCmd(CMD25, sector, 0);

				// CK_SPI_DMA_SetBuffer is called before enabling dma
				// so now just start transfer
				CK_SPI_DMA_Enable(CK_MICROCARD_DMA_STREAM);

				CK_SPI_EnableTXDMA(CK_MICROCARD_SPI);

				card.is_multi_started = true;
        	}
        	else{

        		// SPI will not be deselected until all sectors are written
        		CK_MICROCARD_SelectCard();

				// CK_SPI_DMA_SetBuffer is called before enabling dma
				// so now just start transfer
				CK_SPI_DMA_Enable(CK_MICROCARD_DMA_STREAM);

				CK_SPI_EnableTXDMA(CK_MICROCARD_SPI);
        	}
            break;

        case SPI_DMA_INTERRUPT_SINGLEBLOCK:

            CK_MICROCARD_SelectCard();

            resp = CK_MICROCARD_SendCmd(CMD24, sector, 0);

            // CK_SPI_DMA_SetBuffer is called before enabling dma
            // so now just start transfer
            CK_SPI_DMA_Enable(CK_MICROCARD_DMA_STREAM);

            CK_SPI_EnableTXDMA(CK_MICROCARD_SPI);

            break;

        case SDIO_DMA_INTERRUPT_MULTIBLOCK:

			#if USE_H7 == 1
			// Clean before tx operation when dcache is enabled
			// Buffer is filled by cpu to cache so flush it to sram with cleandcache method for dma to send it to peripheral
			SCB_CleanDCache_by_Addr((uint32_t*)flightLog.log_buffer_1, LOG_BUFFER_SIZE + 512);
			#endif

			HAL_SD_WriteBlocks_DMA(&hsd1, flightLog.log_buffer_1, sector, BLOCK_CACHE_SIZE);

        	break;

        case SPI_POLLING_SINGLEBLOCK:

            CK_MICROCARD_SelectCard();

            resp = CK_MICROCARD_SendCmd(CMD24, sector, 0); // send sector number

            CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFE); // Send Start Token

            for(int i = 0; i < 512; i++){
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, '.');
            }

            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

            // Card deselected at the end of wait busy
            resp = CK_MICROCARD_WaitCardBusy();
            break;

        case SPI_POLLING_MULTIBLOCK:

            // SPI will not be deselected untill whole block of sectors are written
            CK_MICROCARD_SelectCard();

            // This also needed to be send once
            resp = CK_MICROCARD_SendCmd(CMD25, sector, 0);

            //it will write to numOfSector sector at once
            num = card.max_number_of_sector_write;
            while(num--){

                CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFC); // Send Start Token

                for(int i = 0; i < 512; i++){
                    resp = CK_SPI_Transfer(CK_MICROCARD_SPI, '#');
                }

                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

                // Wait card busy
                CK_TIME_SetTimeOut(card.TIME_OUT*10);
                while((CK_MICROCARD_MultiWrite_CheckIsCardBusy() != 0xFF) && CK_TIME_GetTimeOut());

            }

            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFD); // Send Stop Token
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

            CK_MICROCARD_DeselectCard();

            break;

        case SPI_DMA_POLLING_SINGLEBLOCK:

            CK_MICROCARD_SelectCard();

            resp = CK_MICROCARD_SendCmd(CMD24, sector, 0);

            CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFE); // Send Start Token

            // CK_SPI_DMA_SetBuffer is called before enabling dma
            CK_SPI_DMA_Enable(CK_MICROCARD_DMA_STREAM);
            CK_SPI_EnableTXDMA(CK_MICROCARD_SPI);

            CK_TIME_SetTimeOut(card.TIME_OUT * 10);
            while(!CK_SPI_DMA_IsTransferComplete(CK_MICROCARD_DMA, CK_MICROCARD_DMA_STREAM) && CK_TIME_GetTimeOut());

            CK_SPI_DMA_Disable(CK_MICROCARD_DMA_STREAM);
            CK_SPI_DisableTXDMA(CK_MICROCARD_SPI);

            // Send 2byte crc. CRC value does not matter.
            CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
            CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

            // Card deselected at the end of wait busy
            resp = CK_MICROCARD_WaitCardBusy();

            break;

        case SPI_DMA_POLLING_MULTIBLOCK:

            // SPI will not be deselected untill whole block of sectors are written
            CK_MICROCARD_SelectCard();

            // This also needed to be send once
            resp = CK_MICROCARD_SendCmd(CMD25, sector, 0);

            //it will write to numOfSector sector at once
            num = card.max_number_of_sector_write;
            while(num--){

                CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFC); // Send Start Token

                // CK_SPI_DMA_SetBuffer is called before enabling dma
                // so now just start transfer
                CK_SPI_DMA_Enable(CK_MICROCARD_DMA_STREAM);
                CK_SPI_EnableTXDMA(CK_MICROCARD_SPI);

                CK_TIME_SetTimeOut(card.TIME_OUT*10);
                while(!CK_SPI_DMA_IsTransferComplete(CK_MICROCARD_DMA, CK_MICROCARD_DMA_STREAM) && CK_TIME_GetTimeOut());

                CK_SPI_DMA_Disable(CK_MICROCARD_DMA_STREAM);
                CK_SPI_DisableTXDMA(CK_MICROCARD_SPI);

                // Send 2byte crc. CRC value does not matter.
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

                // Wait card busy
                CK_TIME_SetTimeOut(card.TIME_OUT*10);
                while((CK_MICROCARD_MultiWrite_CheckIsCardBusy() != 0xFF) && CK_TIME_GetTimeOut());

            }

            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0x4D); // Send Stop Token
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

            CK_MICROCARD_DeselectCard();
            break;

        default:
            break;

    }

}

void CK_MICROCARD_ReadData(uint32_t sector, uint32_t length, uint8_t* buffer){

#if LOG_SPI_

    uint8_t resp;

    // Single block read
    if(length == 1){

        CK_MICROCARD_SelectCard();

        CK_TIME_SetTimeOut(card.TIME_OUT);
        do{
            // Send single block read command
            resp = CK_MICROCARD_SendCmd(CMD17, sector + card.SECTOR_OFFSET, 0);
        }
        while(resp != 0x00 && CK_TIME_GetTimeOut());

        CK_TIME_SetTimeOut(card.TIME_OUT);
        do{
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        }
        while(resp != 0xFE && CK_TIME_GetTimeOut());

        for(int index = 0; index < 512; index++){

            *buffer++ = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        }

        CK_TIME_SetTimeOut(card.TIME_OUT);
        do{
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF); // CRC
        }
        while(resp != 0xFF && CK_TIME_GetTimeOut());

        CK_MICROCARD_DeselectCard();

    }

    // Multi block read
    // For multi reading i did not implemented array return concept for each sector
    // If it is needed later i will implement.

    else if(length > 1){

        CK_MICROCARD_SelectCard();

        CK_TIME_SetTimeOut(card.TIME_OUT);
        do{
            resp = CK_MICROCARD_SendCmd(CMD18, sector + card.SECTOR_OFFSET, 0); // Send multi block read command
        }
        while(resp != 0x00 && CK_TIME_GetTimeOut());

        CK_TIME_SetTimeOut(card.TIME_OUT);
        do{
            resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        }
        while(resp != 0xFE && CK_TIME_GetTimeOut());

        for(uint32_t current_sector = 0; current_sector < length; current_sector++){

            if(current_sector != 0){
                CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
            }

            for(int index = 0; index < 512; index++){

                *buffer++ = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);

            }

            CK_TIME_SetTimeOut(card.TIME_OUT);
            do{
                resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF); // CRC
            }
            while(resp != 0xFF && CK_TIME_GetTimeOut());

        }

        resp = CK_MICROCARD_SendCmd(CMD12, 0, 0); // Send stop command

        CK_MICROCARD_DeselectCard();

    }

#endif

#if LOG_SDIO_

    HAL_SD_ReadBlocks(&hsd1, buffer, sector + card.SECTOR_OFFSET, length, 1000);


#endif

}

uint8_t CK_MICROCARD_SendAppCommand(uint8_t cmd, uint32_t arg){

    uint8_t resp = CK_MICROCARD_SendCmd(CMD55, 0, 0);

    resp = CK_MICROCARD_SendCmd(cmd, arg, 0);

    return resp;
}

uint8_t CK_MICROCARD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc){

    //CK_MICROCARD_WaitForIdle(8);
    CK_SPI_Transfer(CK_MICROCARD_SPI,0xFF); // Works as well do not spend extra time with 8 spi transfer

    //Send command packet
    CK_SPI_Transfer(CK_MICROCARD_SPI, cmd | 0x40);
    CK_SPI_Transfer(CK_MICROCARD_SPI, arg >> 24);
    CK_SPI_Transfer(CK_MICROCARD_SPI, arg >> 16);
    CK_SPI_Transfer(CK_MICROCARD_SPI, arg >> 8);
    CK_SPI_Transfer(CK_MICROCARD_SPI, arg);
    CK_SPI_Transfer(CK_MICROCARD_SPI, crc); // important for cmd0 and cmd8

    // Command response
    return CK_MICROCARD_WaitForResponse(4);

}

uint8_t CK_MICROCARD_SendStopToken(void){

	// Send the stop token when all sectors are written

	CK_MICROCARD_SelectCard();

	uint8_t resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFD);     // Send Stop Token
	resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);				// resp

    CK_MICROCARD_DeselectCard();

	card.is_multi_started = false;

	return resp;
}

uint8_t CK_MICROCARD_WaitForResponse(int bytesToWait){

    for(int i=0; i < bytesToWait; i++){
        uint8_t response = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        if (response != 0xFF){
            return response;
        }
    }
    return 0xFF;

}

int CK_MICROCARD_WaitForIdle(int bytesToWait){

    while(bytesToWait > 0){
        uint8_t res = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        if (res == 0xFF){
            return 1;
        }
        bytesToWait--;
    }
    return 0;
}

void CK_MICROCARD_SPIFullSpeed(void){

    CK_SPI_WaitTransfer(CK_MICROCARD_SPI);

    uint32_t card_clock = 25000000;

    if(card.card_speed_clock == 50 && card.is_Initialized){
    	card_clock = 50000000;
    }
    else if(card.card_speed_clock == 25 && card.is_Initialized){
    	card_clock = 25000000;
    }

    CK_SPIx_CR1_Fclk_Div clock_rate = CK_SPI_GetClockRate(CK_MICROCARD_SPI, card_clock);

	CK_SPI_ChangeClock(CK_MICROCARD_SPI, clock_rate);

}

void CK_MICROCARD_DeselectCard(void){

	// At least one is needed
    for(int i=0; i < 1; i++){
        uint8_t resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);
        UNUSED(resp);
    }

    CK_GPIO_SetPin(CK_MICROCARD_CS_PORT, CK_MICROCARD_CS_PIN);
}

void CK_MICROCARD_SelectCard(void){

    CK_GPIO_ClearPin(CK_MICROCARD_CS_PORT, CK_MICROCARD_CS_PIN);
}

uint8_t CK_MICROCARD_WaitCardBusy(void){

	uint8_t resp = 0;

#if LOG_SPI_

	CK_MICROCARD_SelectCard();

    CK_TIME_SetTimeOut(card.TIME_OUT);
    while(((resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF)) != 0xFF) && CK_TIME_GetTimeOut());

    CK_MICROCARD_DeselectCard();

    if(resp == 0xFF){
    	resp = HAL_OK;
    }
    else{
    	resp = HAL_ERROR;
    }

#endif

#if LOG_SDIO_
    uint32_t loop = SD_TIMEOUT;

    while(loop > 0)
    {
      loop--;
      if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
      {
          return HAL_OK;
      }
    }

    resp = HAL_ERROR;

#endif

    return resp;

}

uint8_t CK_MICROCARD_CheckIsCardBusy(void){

	uint8_t resp = 0;

	#if LOG_SPI_
    // In single write the card busy will be checked at the end
    // so now card can be deselected as well.

    CK_MICROCARD_SelectCard();

    resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);// 0xFF means OK.

    CK_MICROCARD_DeselectCard();

    if(resp == 0xFF){
    	resp = HAL_OK;
    }
    else{
    	resp = HAL_ERROR;
    }

	#endif

	#if LOG_SDIO_

	if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
	{
		resp = HAL_OK;
	}
	else{
		resp = HAL_ERROR;
	}

	#endif

    return resp;
}

uint8_t CK_MICROCARD_MultiWrite_CheckIsCardBusy(void){

    // In multi write spi will not deactivated until the end of all sectors written

    uint8_t resp = CK_SPI_Transfer(CK_MICROCARD_SPI, 0xFF);// 0xFF means OK.
    return resp;

}

void CK_MICROCARD_WaitTransferComplete(void){

    while(!CK_SPI_DMA_IsTransferComplete(CK_MICROCARD_DMA, CK_MICROCARD_DMA_STREAM));
}

uint16_t CK_MICROCARD_NumberOfDataLeft(void){

    return CK_SPI_DMA_NumberOfDataLeft(CK_MICROCARD_DMA_STREAM);
}

bool CK_MICROCARD_IsLoggingDone(void){
    return card.is_logging_done;
}

bool CK_MICROCARD_IsDMAReady(void){
    return card.is_dma_ready;
}

void CK_MICROCARD_DecrementCurrentSector(uint8_t num){
	card.CURRENT_SECTOR -= num;
}

#if LOG_SDIO_

void SDMMC1_IRQHandler(void){

  HAL_SD_IRQHandler(&hsd1);
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd){

	//SCB_InvalidateDCache_by_Addr((uint32_t*)rx_array, 512);
	//is_rx_dma_done = 1;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd){

	CK_LED_ToggleLed(2);

	card.is_dma_ready = true;  // dma transfer is done

	// Info write tx interrupt
	if(card.is_infoSector_write){

		card.is_infoSector_write = false; // Done writing info sector

	}
	// Data log tx interrupt
	else{

		card.CURRENT_SECTOR += BLOCK_CACHE_SIZE;    // move to the next sector.

		if(card.CURRENT_SECTOR == card.max_number_of_sector_write){

			// Logging done, update will not enter log writing code.
			card.is_logging_done = true;
		}

	}
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd){

	// Check log states etc everything when it enters here
	hsd->ErrorCode;

	// I changed SDMMC_CMDTIMEOUT to 200ms from 5 second in stm32h7xx_II_sdmmc.h line 311
	if(hsd->ErrorCode == 1){


	}

	// After changing the sector from current to info it writes info sector but
	// cannot write next sector properly. I moved cursor to the previous sector
	if(hsd->ErrorCode == 16){

		SDMMC1->ICR |= 1U << 4;

		card.is_dma_ready = true;

		card.CURRENT_SECTOR -= BLOCK_CACHE_SIZE;    // move to the prev sector.
	}

}

#endif

#if LOG_SPI_
void MICROCARD_DMA_TX_Handler(void){

	CK_LED_ToggleLed(2);

    uint8_t resp; UNUSED(resp);

    if(CK_SPI_DMA_IsTransferComplete(CK_MICROCARD_DMA, CK_MICROCARD_DMA_STREAM)){ // Transfer of one sector is done.

    	CK_SPI_DMA_ClearFlag(CK_MICROCARD_DMA, CK_MICROCARD_DMA_STREAM);

        CK_SPI_DMA_Disable(CK_MICROCARD_DMA_STREAM);

        CK_SPI_DisableTXDMA(CK_MICROCARD_SPI);

        card.is_dma_ready = true;           // dma transfer is done

    	// Info write tx interrupt
    	if(card.is_infoSector_write){

    		card.is_infoSector_write = false; // Done writing info sector

    		// If info is written when normal mode is multi take it back to multi again after write of info sectorA
    		if(card.intial_transfer_mode == SPI_DMA_INTERRUPT_MULTIBLOCK){
    			card.transfer_mode = SPI_DMA_INTERRUPT_MULTIBLOCK;
    		}

    		CK_MICROCARD_DeselectCard();
    	}
    	// Data log tx interrupt
    	else{

    		card.CURRENT_SECTOR += BLOCK_CACHE_SIZE;    // move to the next sector.

    		if(card.CURRENT_SECTOR == card.max_number_of_sector_write){

    			// Logging done, update will not enter log writing code.
    			card.is_logging_done = true;

    		}

    		CK_MICROCARD_DeselectCard();

    	}

    }

}

#endif

uint16_t CK_MICROCARD_GetStartByteOfFile(uint8_t* buffer, uint8_t* filename_to_look){

    int states = 0;
    int start_byte_of_file = 0;

    for(int index = 0; index < 512; index++){

        uint8_t current_byte = buffer[index];

        switch(states){
            case 0:
                if(current_byte == filename_to_look[0]){
                    states++;
                    start_byte_of_file = index;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 1:
                if(current_byte == filename_to_look[1]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 2:
                if(current_byte == filename_to_look[2]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 3:
                if(current_byte == filename_to_look[3]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 4:
                if(current_byte == filename_to_look[4]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 5:
                if(current_byte == filename_to_look[5]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 6:
                if(current_byte == filename_to_look[6]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 7:
                if(current_byte == filename_to_look[7]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 8:
                if(current_byte == filename_to_look[8]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 9:
                if(current_byte == filename_to_look[9]){
                    states++;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

            case 10:
                if(current_byte == filename_to_look[10]){
                    states = 0;
                    return start_byte_of_file;
                    break;
                }
                start_byte_of_file = 0;
                states = 0;
                break;

        }
    }

    return 0;

}
