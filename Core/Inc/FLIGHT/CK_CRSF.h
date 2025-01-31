
#ifndef INC_FLIGHT_CK_CRSF_H_
#define INC_FLIGHT_CK_CRSF_H_

#include "CK_DEFINITIONS.h"

typedef enum
{
    CRSF_INTERRUPT,
    CRSF_POLLING,

}CRSF_Method;

typedef enum{
    CRSF_RF_MODE_4HZ = 0,
    CRSF_RF_MODE_50HZ,
    CRSF_RF_MODE_150HZ,
    CRSF_RF_MODE_250HZ,
    CRSF_RF_MODE_UNKNOWN,
}CRSF_RFMode;

#define CRSF_RSSI_MIN (-130)
#define CRSF_RSSI_MAX 0
#define CRSF_SNR_MIN (-30)
#define CRSF_SNR_MAX 20

void CK_CRSF_Init(CRSF_Method method);

bool CK_CRSF_Update(uint32_t current_time);

void CK_CRSF_StartDMA();

void CK_CRSF_SendTelemetry(void);

void CK_CRSF_AddByte(uint8_t index, uint8_t byte);

void CK_CRSF_NewByte(uint8_t data);

void CK_CRSF_NewByte2(uint8_t data);

void CK_CRSF_Decode(void);

void CK_CRSF_Decode_11bit_Channels(void);

void CK_CRSF_Decode_LinkStatusFrame(void);

int CK_CRSF_GetChannelRaw(int channel);

int16_t CK_CRSF_GetRSSI_dBm(void);

CRSF_RFMode CK_CRSF_GetRFMode(void);

uint8_t CK_CRSF_GetLinkQuality(void);

#endif /* INC_FLIGHT_CK_CRSF_H_ */
