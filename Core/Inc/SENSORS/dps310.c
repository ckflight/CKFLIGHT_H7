// ============================================================
// CK_DPS310.c  (DPS310 Baro Driver - CK format + WHO-AM-I + CK_I2C timeout integration)
// ============================================================

#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"

#include "SENSORS/dps310.h"

#include <stdbool.h>
#include <stdint.h>

// ============================================================
// DPS310 I2C Address (7-bit)
// ============================================================
//  - 0x76 if SDO low
#define DPS310_ADDRESS              0x76

// ============================================================
// DPS310 Registers
// ============================================================
#define DPS310_PRS_B2               0x00
#define DPS310_PRS_B1               0x01
#define DPS310_PRS_B0               0x02

#define DPS310_TMP_B2               0x03
#define DPS310_TMP_B1               0x04
#define DPS310_TMP_B0               0x05

#define DPS310_PRS_CFG              0x06
#define DPS310_TMP_CFG              0x07
#define DPS310_MEAS_CFG             0x08
#define DPS310_CFG_REG              0x09

#define DPS310_INT_STS              0x0A
#define DPS310_FIFO_STS             0x0B

#define DPS310_RESET                0x0C
#define DPS310_PROD_ID              0x0D     // PROD_ID/REV_ID (WHO-AM-I style)

#define DPS310_COEF_BASE            0x10     // 0x10..0x21 (18 bytes)
#define DPS310_RESET_CMD            0x89

// ============================================================
// Sizes
// ============================================================
#define DPS310_COEF_READ_SIZE       18
#define DPS310_RAW_READ_SIZE        6   // 3 bytes P + 3 bytes T

// ============================================================
// Internal I2C handle
// ============================================================
static I2C_TypeDef* DPS310_I2C = 0;

// ============================================================
// Parameters struct (CK style)
// ============================================================
typedef struct
{
    bool    BaroInit;

    uint8_t rxArray[DPS310_RAW_READ_SIZE];
    uint8_t coefArray[DPS310_COEF_READ_SIZE];

    // decoded coefficients (signed)
    int16_t  c0;
    int16_t  c1;
    int32_t  c00;
    int32_t  c10;
    int16_t  c01;
    int16_t  c11;
    int16_t  c20;
    int16_t  c21;
    int16_t  c30;

    // OSR codes used (0..7)
    uint8_t prs_osr;
    uint8_t tmp_osr;

    // last raw values (signed 24-bit)
    int32_t prs_raw;
    int32_t tmp_raw;

    // last compensated outputs
    float   pressure_pa;
    float   temperature_c;

}DPS310_PARAMETERS_t;

static DPS310_PARAMETERS_t dps310 =
{
    .BaroInit = false,
    .rxArray = {0,0,0,0,0,0},
    .coefArray = {0},

    .c0 = 0, .c1 = 0,
    .c00 = 0, .c10 = 0,
    .c01 = 0, .c11 = 0, .c20 = 0, .c21 = 0, .c30 = 0,

    .prs_osr = 0,
    .tmp_osr = 0,

    .prs_raw = 0,
    .tmp_raw = 0,

    .pressure_pa = 0.0f,
    .temperature_c = 0.0f
};

// ============================================================
// Local wrappers: integrate CK_I2C timeout counters
// ============================================================

static bool DPS310_I2C_IsReady(I2C_TypeDef* i2c)
{
    // Your CK_I2C_CheckInitialized returns 1 when ready, 0 not init, 2 error
    return (CK_I2C_CheckInitialized(i2c) == 1);
}

static void DPS310_I2C_WriteReg(I2C_TypeDef* i2c, uint8_t reg, uint8_t data)
{
    CK_I2C_Transfer(i2c, DPS310_ADDRESS, reg, data);
    // If your CK_I2C layer increments timeout elsewhere, you can leave this.
    // Otherwise, we at least can reset on writes we assume succeed.
    CK_I2C_ResetTimeOut(i2c);
}

static void DPS310_I2C_Read(I2C_TypeDef* i2c, uint8_t reg, uint8_t* buf, int len)
{
    CK_I2C_ReadMulti(i2c, DPS310_ADDRESS, reg, buf, len);
    //CK_I2C_ResetTimeOut(i2c);
}

// ============================================================
// Helpers
// ============================================================

static int32_t DPS310_SignExtend24(uint32_t x)
{
    if (x & 0x800000U) { x |= 0xFF000000U; }
    return (int32_t)x;
}

static int32_t DPS310_SignExtend20(uint32_t x)
{
    if (x & 0x80000U) { x |= 0xFFF00000U; }
    return (int32_t)x;
}

static int16_t DPS310_SignExtend12(uint16_t x)
{
    if (x & 0x0800U) { x |= 0xF000U; }
    return (int16_t)x;
}

// Consistent scaling table (SHIFT disabled).
// This table is commonly used in DPS310 reference implementations.
static float DPS310_GetScaleFactor(uint8_t osr_code)
{
    switch (osr_code & 0x07U)
    {
        default:
        case 0: return 524288.0f;    // x1
        case 1: return 1572864.0f;   // x2
        case 2: return 3670016.0f;   // x4
        case 3: return 7864320.0f;   // x8
        case 4: return 253952.0f;    // x16   (DPS310 uses an internal shift scheme in some modes; keep SHIFT disabled with this)
        case 5: return 516096.0f;    // x32
        case 6: return 1040384.0f;   // x64
        case 7: return 2088960.0f;   // x128
    }
}

static void DPS310_DecodeCoefficients(void)
{
    const uint8_t* c = dps310.coefArray;

    uint16_t c0_u  = (uint16_t)((((uint16_t)c[0]) << 4) | (c[1] >> 4));
    uint16_t c1_u  = (uint16_t)((((uint16_t)(c[1] & 0x0F)) << 8) | c[2]);

    uint32_t c00_u = (uint32_t)((((uint32_t)c[3]) << 12) | (((uint32_t)c[4]) << 4) | (c[5] >> 4));
    uint32_t c10_u = (uint32_t)((((uint32_t)(c[5] & 0x0F)) << 16) | (((uint32_t)c[6]) << 8) | c[7]);

    dps310.c0  = DPS310_SignExtend12(c0_u);
    dps310.c1  = DPS310_SignExtend12(c1_u);

    dps310.c00 = DPS310_SignExtend20(c00_u);
    dps310.c10 = DPS310_SignExtend20(c10_u);

    dps310.c01 = (int16_t)((((int16_t)c[8])  << 8) | c[9]);
    dps310.c11 = (int16_t)((((int16_t)c[10]) << 8) | c[11]);
    dps310.c20 = (int16_t)((((int16_t)c[12]) << 8) | c[13]);
    dps310.c21 = (int16_t)((((int16_t)c[14]) << 8) | c[15]);
    dps310.c30 = (int16_t)((((int16_t)c[16]) << 8) | c[17]);
}

static void DPS310_Compensate(void)
{
    float prs_sc = (float)dps310.prs_raw / DPS310_GetScaleFactor(dps310.prs_osr);
    float tmp_sc = (float)dps310.tmp_raw / DPS310_GetScaleFactor(dps310.tmp_osr);

    // Temperature (°C)
    dps310.temperature_c = (0.5f * (float)dps310.c0) + ((float)dps310.c1 * tmp_sc);

    // Pressure (Pa)
    float p =
        (float)dps310.c00
        + prs_sc * ((float)dps310.c10 + prs_sc * ((float)dps310.c20 + prs_sc * (float)dps310.c30))
        + tmp_sc * ((float)dps310.c01 + prs_sc * ((float)dps310.c11 + prs_sc * (float)dps310.c21));

    dps310.pressure_pa = p;
}

// WHO-AM-I style check (robust for your CK_I2C with no status return)
static bool DPS310_CheckWhoAmI(I2C_TypeDef* i2c)
{
    uint8_t id = 0x00;

    DPS310_I2C_Read(i2c, DPS310_PROD_ID, &id, 1);

    // Fast reject: bus floating / no device often reads 0x00 or 0xFF.
    if (id == 0x00U || id == 0xFFU) {
        return false;
    }

    // DPS310: upper nibble is product id (0x1), lower nibble revision
    if ( (id & 0xF0U) != 0x10U ) {
        return false;
    }

    return true;
}

// Map your barometer loop time (us) to DPS310 rate code.
// baroFreq: your "targetLoopTime" (likely microseconds).
static uint8_t DPS310_SelectRateCode(uint32_t baroFreq_us)
{
    // Conservative mapping:
    // If you call baro at ~100Hz, pick 64Hz. If ~50Hz pick 32Hz, etc.
    // You can tune this later.
    if (baroFreq_us <= 8000U)   return 6; // <= 8ms  -> 64Hz
    if (baroFreq_us <= 16000U)  return 5; // <=16ms  -> 32Hz
    if (baroFreq_us <= 32000U)  return 4; // <=32ms  -> 16Hz
    if (baroFreq_us <= 64000U)  return 3; // <=64ms  -> 8Hz
    if (baroFreq_us <= 125000U) return 2; // <=125ms -> 4Hz
    if (baroFreq_us <= 250000U) return 1; // <=250ms -> 2Hz
    return 0; // 1Hz
}

// ============================================================
// Public API
// ============================================================

void CK_DPS310_Init(I2C_TypeDef* I2Cn, uint32_t baroFreq)
{
    DPS310_I2C = I2Cn;
    dps310.BaroInit = false;

    // Guard: ensure I2C peripheral was initialized in your system
    if (!DPS310_I2C_IsReady(DPS310_I2C))
    {
        // Optional: beep
        CK_BUZZER_Tone3();
        return;
    }

	// WHO-AM-I check before reset (quick wiring/address validation)
	if (!DPS310_CheckWhoAmI(DPS310_I2C))
	{
		return;
	}

    // Reset
    DPS310_I2C_WriteReg(DPS310_I2C, DPS310_RESET, DPS310_RESET_CMD);
    CK_TIME_DelayMilliSec(10);

    // Optional: WHO-AM-I check again after reset (some boards prefer this)
    if (!DPS310_CheckWhoAmI(DPS310_I2C))
    {
        return;
    }

    // Read coefficients
    DPS310_I2C_Read(DPS310_I2C, DPS310_COEF_BASE, dps310.coefArray, DPS310_COEF_READ_SIZE);
    DPS310_DecodeCoefficients();

    // --------------------------------------------------------
    // Configure DPS310
    // --------------------------------------------------------
    uint8_t rate_code = DPS310_SelectRateCode(baroFreq); // based on your loop time
    uint8_t osr_code  = 3; // x8 default (good tradeoff)

    dps310.prs_osr = osr_code;
    dps310.tmp_osr = osr_code;

    // PRS_CFG: [7:4]=rate, [3:0]=osr
    DPS310_I2C_WriteReg(DPS310_I2C, DPS310_PRS_CFG, (uint8_t)((rate_code << 4) | (osr_code & 0x0F)));

    // TMP_CFG: [7:4]=rate, [3:0]=osr
    // Also TMP_EXT bit exists on some DPS3xx variants; keeping default internal temp.
    DPS310_I2C_WriteReg(DPS310_I2C, DPS310_TMP_CFG, (uint8_t)((rate_code << 4) | (osr_code & 0x0F)));

    // CFG_REG:
    // - Keep SHIFT disabled for simplicity (0x00)
    // - If you later use high OSR (>= x64), consider enabling shift and adjusting scale factors.
    DPS310_I2C_WriteReg(DPS310_I2C, DPS310_CFG_REG, 0x00);

    // MEAS_CFG:
    // 0x05 = continuous pressure + temperature
    DPS310_I2C_WriteReg(DPS310_I2C, DPS310_MEAS_CFG, 0x05);

    dps310.BaroInit = true;
}

void CK_DPS310_ReadBaroRaw(void)
{
    if (!dps310.BaroInit || DPS310_I2C == 0) {
        return;
    }

    DPS310_I2C_Read(DPS310_I2C, DPS310_PRS_B2, dps310.rxArray, DPS310_RAW_READ_SIZE);

    uint32_t prs_u = ((uint32_t)dps310.rxArray[0] << 16)
                   | ((uint32_t)dps310.rxArray[1] << 8)
                   |  (uint32_t)dps310.rxArray[2];

    uint32_t tmp_u = ((uint32_t)dps310.rxArray[3] << 16)
                   | ((uint32_t)dps310.rxArray[4] << 8)
                   |  (uint32_t)dps310.rxArray[5];

    dps310.prs_raw = DPS310_SignExtend24(prs_u);
    dps310.tmp_raw = DPS310_SignExtend24(tmp_u);
}

void CK_DPS310_ReadBaro(void)
{
    CK_DPS310_ReadBaroRaw();
    DPS310_Compensate();
}

float CK_DPS310_GetPressurePa(void)
{
    return dps310.pressure_pa;
}

float CK_DPS310_GetTemperatureC(void)
{
    return dps310.temperature_c;
}

bool CK_DPS310_isBaroSensorInitialized(void)
{
    return dps310.BaroInit;
}
