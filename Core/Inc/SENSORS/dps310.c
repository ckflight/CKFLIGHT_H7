// ============================================================
// CK_DPS310.c  (DPS310 Baro Driver - CK format, Adafruit-aligned)
// ============================================================

#include "DRIVERS/CK_I2C.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_BUZZER.h"
#include <stdbool.h>
#include <stdint.h>

// -------------------- I2C address (7-bit) --------------------
#define DPS310_ADDRESS              0x76  // SDO low -> 0x76, SDO high -> 0x77

// -------------------- Registers --------------------
#define DPS310_PRS_B2               0x00
#define DPS310_TMP_B2               0x03

#define DPS310_PRS_CFG              0x06
#define DPS310_TMP_CFG              0x07
#define DPS310_MEAS_CFG             0x08
#define DPS310_CFG_REG              0x09

#define DPS310_RESET                0x0C
#define DPS310_PROD_ID              0x0D

#define DPS310_COEF_BASE            0x10  // 0x10..0x21 (18 bytes)
#define DPS310_TMPCOEFSRCE          0x28  // bit7 source used by Adafruit

#define DPS310_RESET_CMD            0x89

// -------------------- MEAS_CFG bits (Adafruit) --------------------
#define DPS310_BIT_CALIB_RDY        (1u << 7)
#define DPS310_BIT_SENSOR_RDY       (1u << 6)
#define DPS310_BIT_TMP_RDY          (1u << 5)
#define DPS310_BIT_PRS_RDY          (1u << 4)

// -------------------- CFG_REG shift bits (Adafruit) -------------
#define DPS310_BIT_P_SHIFT          (1u << 2)
#define DPS310_BIT_T_SHIFT          (1u << 3)

// -------------------- sizes --------------------
#define DPS310_COEF_READ_SIZE       18
#define DPS310_RAW3_SIZE            3

// -------------------- oversample scaling table (Adafruit) -------
static const int32_t dps310_osr_scale[8] = {
    524288, 1572864, 3670016, 7864320,
    253952, 516096, 1040384, 2088960
};

// -------------------- internal state ----------------------------
static I2C_TypeDef* DPS310_I2C = 0;

typedef struct {
    bool BaroInit;

    // calib
    int16_t  c0, c1;
    int32_t  c00, c10;
    int16_t  c01, c11, c20, c21, c30;

    // config
    uint8_t  prs_osr;        // 0..7
    uint8_t  tmp_osr;        // 0..7
    int32_t  prs_scale;
    int32_t  tmp_scale;

    // last
    int32_t  raw_p;
    int32_t  raw_t;
    float    pressure_pa;
    float    temperature_c;
    float    scaled_rawtemp;

} dps310_t;

static dps310_t s = {0};

// -------------------- helpers --------------------
static int32_t twosComplement(int32_t val, uint8_t bits)
{
    if (val & ((uint32_t)1u << (bits - 1))) {
        val -= (uint32_t)1u << bits;
    }
    return val;
}

static void i2c_write(uint8_t reg, uint8_t v)
{
    CK_I2C_Transfer(DPS310_I2C, DPS310_ADDRESS, reg, v);
}

static void i2c_read(uint8_t reg, uint8_t* buf, int len)
{
    CK_I2C_ReadMulti(DPS310_I2C, DPS310_ADDRESS, reg, buf, len);
}

static uint8_t read_u8(uint8_t reg)
{
    uint8_t v = 0;
    i2c_read(reg, &v, 1);
    return v;
}

static void wait_ready(uint8_t mask, uint32_t timeout_ms)
{
    // simple polling helper
    uint32_t t0 = CK_TIME_GetMicroSec();
    while (1) {
        uint8_t m = read_u8(DPS310_MEAS_CFG);
        if ((m & mask) == mask) return;

        if (((CK_TIME_GetMicroSec() - t0) / 1000u) > timeout_ms) return;
        CK_TIME_DelayMilliSec(1);
    }
}

static bool check_whoami(void)
{
    uint8_t id = read_u8(DPS310_PROD_ID);
    // Adafruit expects exactly 0x10
    return (id == 0x10);
}

static void read_calibration(void)
{
    // Wait CALIB_RDY
    wait_ready(DPS310_BIT_CALIB_RDY, 100);

    uint8_t c[DPS310_COEF_READ_SIZE];
    i2c_read(DPS310_COEF_BASE, c, DPS310_COEF_READ_SIZE);

    int32_t c0  = ((uint16_t)c[0] << 4) | ((c[1] >> 4) & 0x0F);
    int32_t c1  = (((uint16_t)c[1] & 0x0F) << 8) | c[2];

    int32_t c00 = ((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | ((c[5] >> 4) & 0x0F);
    int32_t c10 = (((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | (uint32_t)c[7];

    s.c0  = (int16_t)twosComplement(c0, 12);
    s.c1  = (int16_t)twosComplement(c1, 12);
    s.c00 = (int32_t)twosComplement(c00, 20);
    s.c10 = (int32_t)twosComplement(c10, 20);

    s.c01 = (int16_t)twosComplement(((uint16_t)c[8]  << 8) | c[9],  16);
    s.c11 = (int16_t)twosComplement(((uint16_t)c[10] << 8) | c[11], 16);
    s.c20 = (int16_t)twosComplement(((uint16_t)c[12] << 8) | c[13], 16);
    s.c21 = (int16_t)twosComplement(((uint16_t)c[14] << 8) | c[15], 16);
    s.c30 = (int16_t)twosComplement(((uint16_t)c[16] << 8) | c[17], 16);
}

static void configure_pressure(uint8_t rate_code, uint8_t osr_code)
{
    // PRS_CFG: rate[6:4], osr[3:0]
    uint8_t v = (uint8_t)((rate_code & 0x07u) << 4) | (osr_code & 0x0Fu);
    i2c_write(DPS310_PRS_CFG, v);

    // SHIFT handling (Adafruit): if osr > 8 samples => enable P_SHIFT
    uint8_t cfg = read_u8(DPS310_CFG_REG);
    if (osr_code > 3) cfg |= DPS310_BIT_P_SHIFT;
    else              cfg &= (uint8_t)~DPS310_BIT_P_SHIFT;
    i2c_write(DPS310_CFG_REG, cfg);

    s.prs_osr   = osr_code & 0x07u;
    s.prs_scale = dps310_osr_scale[s.prs_osr];
}

static void configure_temperature(uint8_t rate_code, uint8_t osr_code)
{
    // TMP_CFG: rate[6:4], osr[3:0]
    uint8_t v = (uint8_t)((rate_code & 0x07u) << 4) | (osr_code & 0x0Fu);

    // Copy temp coeff source bit7 into TMP_CFG bit7 (Adafruit behavior)
    uint8_t src = read_u8(DPS310_TMPCOEFSRCE);
    if (src & 0x80u) v |= 0x80u;
    else             v &= 0x7Fu;

    i2c_write(DPS310_TMP_CFG, v);

    // SHIFT handling (Adafruit): if osr > 8 samples => enable T_SHIFT
    uint8_t cfg = read_u8(DPS310_CFG_REG);
    if (osr_code > 3) cfg |= DPS310_BIT_T_SHIFT;
    else              cfg &= (uint8_t)~DPS310_BIT_T_SHIFT;
    i2c_write(DPS310_CFG_REG, cfg);

    s.tmp_osr   = osr_code & 0x07u;
    s.tmp_scale = dps310_osr_scale[s.tmp_osr];
}

// -------------------- public API --------------------
void CK_DPS310_Init(I2C_TypeDef* I2Cn, uint32_t baroLoopTime_us)
{
    (void)baroLoopTime_us;

    DPS310_I2C = I2Cn;
    s.BaroInit = false;

    if (CK_I2C_CheckInitialized(DPS310_I2C) != 1) {
        CK_BUZZER_Tone3();
        return;
    }

    if (!check_whoami()) {
        // wrong address / wiring
        return;
    }

    // Reset
    i2c_write(DPS310_RESET, DPS310_RESET_CMD);
    CK_TIME_DelayMilliSec(10);

    // Wait SENSOR_RDY after reset (Adafruit)
    wait_ready(DPS310_BIT_SENSOR_RDY, 100);

    // Read calib
    read_calibration();

    // Adafruit defaults: 64Hz, 64 samples (osr=6)
    // rate_code: 64Hz is 6 in DPS310 (matches Adafruit enum)
    // osr_code : 64 samples is 6
    configure_pressure(6, 6);
    configure_temperature(6, 6);

    // Continuous pressure+temperature mode = 0x07? (Adafruit uses DPS310_CONT_PRESTEMP)
    // In their code they set modebits(3 bits, pos0). For DPS310_CONT_PRESTEMP it is 7.
    // Many examples also use 0x07. If your board only works with 0x05, keep 0x05.
    // We'll match Adafruit:
    uint8_t meas = read_u8(DPS310_MEAS_CFG);
    meas = (uint8_t)((meas & ~0x07u) | 0x07u);
    i2c_write(DPS310_MEAS_CFG, meas);

    // Wait first data ready (Adafruit loop)
    // temperatureAvailable() bit5, pressureAvailable() bit4
    wait_ready((uint8_t)(DPS310_BIT_TMP_RDY | DPS310_BIT_PRS_RDY), 200);

    s.BaroInit = true;
}

bool CK_DPS310_isBaroSensorInitialized(void)
{
    return s.BaroInit;
}

void CK_DPS310_ReadBaro(void)
{
    if (!s.BaroInit || DPS310_I2C == 0) return;

    // Optional: only update when new data is ready
    uint8_t m = read_u8(DPS310_MEAS_CFG);
    if (((m & DPS310_BIT_TMP_RDY) == 0u) || ((m & DPS310_BIT_PRS_RDY) == 0u)) {
        return;
    }

    uint8_t pb[3], tb[3];
    i2c_read(DPS310_PRS_B2, pb, 3);
    i2c_read(DPS310_TMP_B2, tb, 3);

    int32_t raw_p = ((int32_t)pb[0] << 16) | ((int32_t)pb[1] << 8) | (int32_t)pb[2];
    int32_t raw_t = ((int32_t)tb[0] << 16) | ((int32_t)tb[1] << 8) | (int32_t)tb[2];

    s.raw_p = twosComplement(raw_p, 24);
    s.raw_t = twosComplement(raw_t, 24);

    // Adafruit math
    s.scaled_rawtemp = (float)s.raw_t / (float)s.tmp_scale;
    s.temperature_c  = s.scaled_rawtemp * (float)s.c1 + ((float)s.c0 / 2.0f);

    float p_sc = (float)s.raw_p / (float)s.prs_scale;

    s.pressure_pa =
        (float)s.c00 +
        p_sc * ((float)s.c10 + p_sc * ((float)s.c20 + p_sc * (float)s.c30)) +
        s.scaled_rawtemp * ((float)s.c01 + p_sc * ((float)s.c11 + p_sc * (float)s.c21));
}

float CK_DPS310_GetPressurePa(void)
{
    return s.pressure_pa;
}

float CK_DPS310_GetTemperatureC(void)
{
    return s.temperature_c;
}
