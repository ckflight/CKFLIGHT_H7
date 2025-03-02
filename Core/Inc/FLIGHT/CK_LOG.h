
#ifndef FLIGHT_CK_LOG_H_
#define FLIGHT_CK_LOG_H_

#if (LOG_SPI_ == 0 && LOG_SDIO_ == 0 && LOG_FLASH_==0)
#define LOG_BUFFER_SIZE			0
#define INFO_BUFFER_SIZE		0
#define WRITE_INFO_SECTOR		0
#define BLOCK_CACHE_SIZE		0
#define BYTES_PER_LOG			0
#endif

#if LOG_SPI_

#if LOG_MULTIWRITE_ == 1

#define BLOCK_CACHE_SIZE		1
#define LOG_BUFFER_SIZE			((1 + 512 + 2 + 1)  * BLOCK_CACHE_SIZE)
#define INFO_BUFFER_SIZE		(1 + 512 + 2 + 1)

#define WRITE_INFO_SECTOR		512

#define BYTES_PER_LOG			128

#else

#define BLOCK_CACHE_SIZE		1
#define LOG_BUFFER_SIZE			(1 + 512 + 2 + 1) + 9
#define INFO_BUFFER_SIZE		(1 + 512 + 2 + 1) + 9

#define WRITE_INFO_SECTOR		512

#define BYTES_PER_LOG			128

#endif
#endif

#if LOG_SDIO_

#define BLOCK_CACHE_SIZE		8
#define LOG_BUFFER_SIZE			(512 * BLOCK_CACHE_SIZE)
#define INFO_BUFFER_SIZE		512

#define WRITE_INFO_SECTOR		512

#define BYTES_PER_LOG			128
#endif

#if LOG_FLASH_

#define BLOCK_CACHE_SIZE		1
#define LOG_BUFFER_SIZE		   (512 * BLOCK_CACHE_SIZE)
#define INFO_BUFFER_SIZE		512

#define WRITE_INFO_SECTOR	    128

#define BYTES_PER_LOG			128
#endif


typedef enum{
	BUFFER1,
	BUFFER2
}buffers_e;

typedef struct{

	#if LOG_SPI_
	uint8_t log_buffer_1[LOG_BUFFER_SIZE];
	uint8_t info_buffer[INFO_BUFFER_SIZE];
	#endif

	#if LOG_SDIO_
	uint8_t log_buffer_1[LOG_BUFFER_SIZE] __attribute__ ((aligned (4)));
	uint8_t info_buffer[INFO_BUFFER_SIZE] __attribute__ ((aligned (4)));
	#endif

	#if LOG_FLASH_
	uint8_t log_buffer_1[LOG_BUFFER_SIZE];
	uint8_t info_buffer[INFO_BUFFER_SIZE];
	#endif

	#if (LOG_SPI_ == 0 && LOG_SDIO_ == 0 && LOG_FLASH_==0)
	uint8_t log_buffer_1[LOG_BUFFER_SIZE];
	uint8_t info_buffer[INFO_BUFFER_SIZE];
	#endif

	uint32_t buffer_index;

	uint8_t log_start_byte;
	uint8_t log_end_byte;

	uint32_t info_sector_write_counter;
	bool is_info_write;

	uint32_t log_mainTargetTime;
	uint32_t log_logTargetTime;
	uint8_t syncRate;
	uint32_t log_target_counter;

	uint32_t log_end_led_counter;

	uint32_t log_delay_counter;

}log_parameters_t;

extern log_parameters_t flightLog;

void CK_LOG_Init(uint32_t mainT, uint32_t logT);

void CK_LOG_Update(uint32_t currentLoopTime);

void CK_LOG_WriteInfoBuffer(void);

uint32_t CK_LOG_GetTimeOutStart(void);

#endif
