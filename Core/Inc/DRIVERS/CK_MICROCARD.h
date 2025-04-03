

#ifndef CK_MICROCARD_H_
#define CK_MICROCARD_H_

#include "CK_DEFINITIONS.h"

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

typedef enum{
	SPI_POLLING_SINGLEBLOCK,
	SPI_POLLING_MULTIBLOCK,

	SPI_DMA_POLLING_SINGLEBLOCK,
	SPI_DMA_POLLING_MULTIBLOCK,

	SPI_DMA_INTERRUPT_SINGLEBLOCK,
	SPI_DMA_INTERRUPT_MULTIBLOCK,

	SDIO_DMA_INTERRUPT_MULTIBLOCK


}microcard_transfer_modes_e;

typedef enum{

	IDLE,
	CHECK_CARD_BUSY,
	CHECK_CARD_BUSY1,
	CHECK_CARD_BUSY2,
	FINISH_LOGGING_MULTI_WRITE,
	START_INFOSECTOR_SINGLE_WRITE,
	FINISH_INFOSECTOR_WRITE

}microcard_finish_modes_e;

typedef struct{

	uint8_t init_retry;

	bool is_card_fast;
	bool is_Initialized;
	bool is_dma_ready;
	bool is_card_high_capacity;

	uint8_t card_version;
	uint8_t card_speed_clock;

	uint32_t START_SECTOR;
	uint32_t CURRENT_SECTOR;
	uint32_t SECTOR_OFFSET;
	uint32_t INFO_SECTOR;

	bool is_multi_started;
	uint32_t max_number_of_sector_write;
	bool is_logging_done;
	bool is_infoSector_write;

	uint32_t TIME_OUT;

	microcard_transfer_modes_e transfer_mode;
	microcard_transfer_modes_e intial_transfer_mode; // multi mode changes mode to single when writing info so initial mode must be known

}microcard_parameters_t;

extern microcard_parameters_t card;

extern SD_HandleTypeDef hsd1;

void CK_MICROCARD_Init(microcard_transfer_modes_e mode);

void CK_MICROCARD_InitCard(void);

bool CK_MICROCARD_ReadOffset(void);

void CK_MICROCARD_SaveOffset(uint32_t sector_offset);

void CK_MICROCARD_AccessCardDetails(void);

void CK_MICROCARD_WriteInfoSector(void);

void CK_MICROCARD_WriteData(uint32_t sector);

void CK_MICROCARD_ReadData(uint32_t sector, uint32_t length, uint8_t* buffer);

void CK_MICROCARD_SendByte(uint8_t data);

uint8_t CK_MICROCARD_SendAppCommand(uint8_t cmd, uint32_t arg);

uint8_t CK_MICROCARD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc);

void CK_MICROCARD_SendCmdNoResp(uint8_t cmd, uint32_t arg, uint8_t crc);

uint8_t CK_MICROCARD_SendStopToken(void);

uint8_t CK_MICROCARD_WaitForResponse(int bytesToWait);

int CK_MICROCARD_WaitForIdle(int bytesToWait);

void CK_MICROCARD_SPIFullSpeed(void);

void CK_MICROCARD_DeselectCardNoTransfer(void);

void CK_MICROCARD_DeselectCard(void);

void CK_MICROCARD_SelectCard(void);

uint8_t CK_MICROCARD_WaitCardBusy(void);

uint8_t CK_MICROCARD_CheckIsCardBusy(void);

uint8_t CK_MICROCARD_MultiWrite_CheckIsCardBusy(void);

void CK_MICROCARD_WaitTransferComplete(void);

uint16_t CK_MICROCARD_NumberOfDataLeft(void);

bool CK_MICROCARD_IsLoggingDone(void);

bool CK_MICROCARD_IsDMAReady(void);

void CK_MICROCARD_DecrementCurrentSector(uint8_t num);

uint16_t CK_MICROCARD_GetStartByteOfFile(uint8_t* buffer, uint8_t* filename_to_look);

#endif /* CK_MICROCARD_H_ */
