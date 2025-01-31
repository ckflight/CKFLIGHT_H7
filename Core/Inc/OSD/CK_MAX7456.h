
#ifndef INC_SENSORS_CK_MAX7456_H_
#define INC_SENSORS_CK_MAX7456_H_

#include "CK_DEFINITIONS.h"

typedef enum{
    RSSI_SYMBOL1,       // There is 2 different type rssi symbol
    RSSI_SYMBOL2,
    CELCIUS_SYMBOL,
    SATELITE1_SYMBOL,   // Satelite symbol is divided into 2 symbols
    SATELITE2_SYMBOL,
    NORTH_SYMBOL,
    SOUTH_SYMBOL,
    EAST_SYMBOL,
    WEST_SYMBOL,
    AMPER_SYMBOL,
    VOLTAGE_SYMBOL,
    MAH_SYMBOL,
    METERS_SYMBOL,      // m
    MS_SYMBOL,          // m per second
    CM_SYMBOL,          // cm
    LIPO_0,
    LIPO_16,
    LIPO_32,
    LIPO_48,
    LIPO_64,
    LIPO_80,
    LIPO_100,
    LIPO_BAT,
    HEADING_ARROW_0,
    HEADING_ARROW_23,
    HEADING_ARROW_45,
    HEADING_ARROW_67,
    HEADING_ARROW_90,
    HEADING_ARROW_113,
    HEADING_ARROW_135,
    HEADING_ARROW_157,
    HEADING_ARROW_180,
    HEADING_ARROW_202,
    HEADING_ARROW_225,
    HEADING_ARROW_247,
    HEADING_ARROW_270,
    HEADING_ARROW_292,
    HEADING_ARROW_315,
    HEADING_ARROW_336

}MAX7456_Symbols;


void CK_MAX7456_Init(SPI_TypeDef* spi_, GPIO_TypeDef* gpio_, uint8_t pin_);

void CK_MAX7456_Config(void);

void CK_MAX7456_ShowFonts(void);

uint8_t CK_MAX7456_CharUpdate(void);

void CK_MAX7456_WriteChar(uint8_t ch, const uint8_t* addr);

void CK_MAX7456_Update_DMA(void);

void CK_MAX7456_Update(void);

void CK_MAX7456_TimerPlot(uint32_t current_time);

void CK_MAX7456_VoltagePlot(uint32_t current_time);

void CK_MAX7456_CurrentPlot(uint32_t current_time);

void CK_MAX7456_MahPlot(uint32_t current_time);

void CK_MAX7456_LipoIconPlot(uint32_t current_time);

void CK_MAX7456_FirmwareRatePlot(uint32_t current_time);

void CK_MAX7456_RssiPlot(uint32_t current_time);

void CK_MAX7456_RssidBmPlot(uint32_t current_time);

void CK_MAX7456_RssiLinkQualityPlot(uint32_t current_time);

void CK_MAX7456_GpsSattelitePlot(uint32_t current_time);

void CK_MAX7456_GpsDistanceToDestinationPlot(uint32_t current_time);

void CK_MAX7456_GpsGroundSpeedPlot(uint32_t current_time);

void CK_MAX7456_GpsHeadingToDestinationPlot(uint32_t current_time);

void CK_MAX7456_GpsHeadingOfMotionPlot(uint32_t current_time);

void CK_MAX7456_CoreTemperaturePlot(uint32_t current_time);

void CK_MAX7456_AltitudePlot(uint32_t current_time);

void CK_MAX7456_PidPlot(uint32_t current_time);

void CK_MAX7456_TPAPlot(uint32_t current_time);

void CK_MAX7456_ImuHeadingPlot(uint32_t current_time);

void CK_MAX7456_FlightModePlot(uint32_t current_time);

void CK_MAX7456_AltitudeModePlot(uint32_t current_time);

void CK_MAX746_NavigationModePlot(uint32_t current_time);

void CK_MAX7456_FailsafePlot(uint32_t current_time);

void CK_MAX7456_CKFLIGHTPlot(uint32_t current_time);

void CK_MAX7456_PrintLogo(void);

void CK_MAX7456_ClearLogo(void);

void CK_MAX7456_Write_BufferElements(int startElement, int lastElement);

void CK_MAX7456_ClearBuffer(void);

void CK_MAX7456_OSD_DMA_Packet(int start_address, int osdRow, int osdColumn, const char str[], int len);

void CK_MAX7456_OSD_FillBuffer(int osdRow, int osdColumn, const char str[], int len);

void CK_MAX7456_ClearScreen(void);

void CK_MAX7456_Reset(void);

void CK_MAX7456_CheckReset(uint32_t current_ms);

uint8_t CK_MAX7456_WriteRegister(uint8_t reg, uint8_t data);

int CK_MAX7456_GetPlotIndex(int line, int orientation, int space);

uint8_t CK_MAX7456_GetLipoAddress(MAX7456_Symbols symbol);

uint8_t CK_MAX7456_GetCharacterAddress(int character);

uint8_t CK_MAX7456_GetSpecialCharacterAddress(MAX7456_Symbols symbol);

uint8_t CK_MAX7456_GetArrowCharacter(int angle);

#endif /* INC_SENSORS_CK_MAX7456_H_ */
