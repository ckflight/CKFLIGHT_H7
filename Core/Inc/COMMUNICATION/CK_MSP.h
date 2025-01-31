
#ifndef CK_MSP_H_
#define CK_MSP_H_

#include "CK_DEFINITIONS.h"

#define MSP_DP_HEARTBEAT		0   // Release the display after clearing and updating
#define MSP_DP_RELEASE			1   // Release the display after clearing and updating
#define MSP_DP_CLEAR_SCREEN		2   // Clear the display
#define MSP_DP_WRITE_STRING		3   // Write a string at given coordinates
#define MSP_DP_DRAW_SCREEN		4   // Trigger a screen draw
#define MSP_DP_OPTIONS			5   // Not used by Betaflight. Reserved by Ardupilot and INAV
#define MSP_DP_SYS				6	// Display system element displayportSystemElement_e at given coordinates

#define MSP_API_VERSION			1 // affects if i select 1 42 current betafligh it does not init
#define MSP_FC_VARIANT			2
#define MSP_FC_VERSION			3
#define MSP_BOARD_INFO			4
#define MSP_BUILD_INFO			5
#define MSP_NAME                10

#define MSP_BATTERY_CONFIG		32
#define MSP_FEATURE_CONFIG	  	36 // affects pid page
#define MSP_MIXER_CONFIG	  	42
#define MSP_PID_CONTROLLER	  	59
#define MSP_DATAFLASH_SUMMARY  	70
#define MSP_OSD_CONFIG          84 // out message         Get osd settings - betaflight
#define MSP_FILTER_CONFIG	  	92
#define MSP_PID_ADVANCED		94

#define MSP_STATUS  			101
#define MSP_STATUS_EX  			150

#define MSP_RC				  	105
#define MSP_RAW_GPS			  	106
#define MSP_ATTITUDE		  	108
#define MSP_ANALOG			  	110
#define MSP_RC_TUNING		  	111
#define MSP_PID				  	112
#define MSP_PIDNAMES		  	117
#define MSP_BOXIDS  			119
#define MSP_RC_DEADBAND		  	125

#define MSP_BATTERY_STATE		130

#define MSP_GPS_RESCUE_PIDS		136

#define MSP_UID					160

#define MSP_DISPLAYPORT         182
#define MSP_COPY_PROFILE		183
#define MSP_SET_PID 			202

#define MSP_TX_INFO				187
#define MSP_RTC					246
#define MSP_SET_RTC				247

typedef struct msp_parameters {
    uint8_t msp_buffer[128];
    uint16_t msp_buffer_idx;

} msp_parameters_t;

extern msp_parameters_t msp;

void CK_MSP_DecodeInputStream(uint8_t* buffer, uint16_t size);

uint8_t CK_MSP_GetMessage(uint8_t* buffer, uint16_t length, uint8_t* commandPayloadBuffer, uint16_t* len);

uint16_t CK_MSP_OSDPlotCommands(uint8_t command_id, uint8_t* buffer);

uint16_t CK_MSP_PlotOSD(uint8_t row, uint8_t col, uint8_t* buffer, const char str[], uint8_t len);

uint16_t CK_MSP_FormPacket(uint8_t command_id, uint8_t* buffer);

void CK_MSP_ProcessCommand(uint8_t command_id, uint8_t* commandBuf, uint16_t payloadSize);

void CK_MSP_WriteBufferU8(uint8_t * buffer, uint8_t data, uint8_t idx, uint8_t* crc);

void CK_MSP_WriteBufferU16(uint8_t * buffer, uint16_t data, uint8_t idx, uint8_t* crc);

void CK_MSP_WriteBufferU32(uint8_t * buffer, uint16_t data, uint8_t idx, uint8_t* crc);

void CK_MSP_WriteU8(uint8_t data, uint8_t* crc);

void CK_MSP_WriteU16(uint16_t data, uint8_t* crc);

void CK_MSP_WriteU32(uint32_t data, uint8_t* crc);

void CK_MSP_SendResponse(uint8_t resp);

#endif /* CK_MSP_H_ */
