
#ifndef INC_COMMUNICATION_CK_CONFIGURATION_H_
#define INC_COMMUNICATION_CK_CONFIGURATION_H_

#include "CK_DEFINITIONS.h"

#define CONFIG_ID				"CK<$"
#define CONFIG_ID_OFFSET		0
#define CONFIG_ID_BYTES			20 // 20 Bytes allocated

#define CONFIG_PID_OFFSET		CONFIG_ID_OFFSET + CONFIG_ID_BYTES
#define CONFIG_PID_BYTES		40 // 40 Bytes

#define CONFIG_ACC_OFFSET		CONFIG_PID_OFFSET + CONFIG_PID_BYTES
#define CONFIG_ACC_BYTES		4  // 4 Bytes

#define CONFIG_RC_OFFSET		CONFIG_ACC_OFFSET + CONFIG_ACC_BYTES
#define CONFIG_RC_BYTES			30 // 30 Bytes

#define CONFIG_SD_OFFSET		CONFIG_RC_OFFSET + CONFIG_RC_BYTES
#define CONFIG_SD_BYTES			10 // 10 Bytes

void CK_CONFIGURATION_Init(void);

void CK_CONFIGURATION_DecodeInputStream(uint8_t* buffer, uint16_t buffer_size);

void CK_CONFIGURATION_StartCMD(void);

uint8_t CK_CONFIGURATION_ConfigureParameters(void);

uint16_t CK_CONFIGURATION_AsciiToNumber(uint8_t* buffer, int start, int end);

void CK_CONFIGURATION_LoadID(void);

void CK_CONFIGURATION_LoadParameters(void);

void CK_CONFIGURATION_SavePIDs(int axis, uint16_t* pid_buffer, int pid_buffer_size);

void CK_CONFIGURATION_SaveAccCalibration(int16_t* acc_buffer);

void CK_CONFIGURATION_SaveRC(int option, uint16_t num);

void CK_CONFIGURATION_SaveRates(int option, uint16_t* rate_buffer, int rate_buffer_size);













#endif /* INC_COMMUNICATION_CK_CONFIGURATION_H_ */
