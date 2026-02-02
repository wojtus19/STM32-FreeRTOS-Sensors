/*
 * DistanceSensor.c
 *
 * Driver for VL53L0X Time-of-Flight Distance Sensor
 * This code is based on
 * https://github.com/pololu/vl53l0x-arduino/tree/master
 *
 * Ported to STM32 Cube HAL by Wojciech Niewiadomski
 *
 *  Created on: Jan 25, 2026
 *
 */

// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "DistanceSensor.h"
#include "i2c_manager.h"
#include "logger.h"
#include "typedef.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = HAL_GetTick())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)(HAL_GetTick() - timeout_start_ms) > io_timeout))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

typedef enum regAddr_t
{
    SYSRANGE_START                              = 0x00,
    SYSTEM_THRESH_HIGH                          = 0x0C,
    SYSTEM_THRESH_LOW                           = 0x0E,
    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_RANGE_CONFIG                         = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
    ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
    MSRC_CONFIG_CONTROL                         = 0x60,
    PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
    FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
    SYSTEM_HISTOGRAM_BIN                        = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
    SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    IDENTIFICATION_REVISION_ID                  = 0xC2,
    OSC_CALIBRATE_VAL                           = 0xF8,
    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,
    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,
    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
} regAddr_t;

typedef struct SequenceStepEnables_t
{
    bool_t tcc;
    bool_t msrc;
    bool_t dss;
    bool_t pre_range;
    bool_t final_range;
} SequenceStepEnables_t;

typedef struct SequenceStepTimeouts_t
{
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} SequenceStepTimeouts_t;

static uint16_t io_timeout;
static bool_t did_timeout;
static uint16_t timeout_start_ms;
static uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
static uint32_t measurement_timing_budget_us;

static Status_t ReadRegister(uint8_t reg, uint8_t* data);
static Status_t ReadRegister16Bit(uint8_t reg, uint16_t* value);
static Status_t ReadMulti(uint8_t reg, uint8_t* dst, uint8_t count);
static Status_t WriteRegister16Bit(uint8_t reg, uint16_t value);
static Status_t WriteRegister32Bit(uint8_t reg, uint32_t value);
static Status_t WriteRegister(uint8_t reg, uint8_t value);
static Status_t WriteMulti(uint8_t reg, uint8_t* src, uint8_t count);
static Status_t InitSensor(uint8_t io_2v8);
static uint32_t GetMeasurementTimingBudget();
static uint8_t SetMeasurementTimingBudget(uint32_t budget_us);
static uint32_t TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static uint16_t EncodeTimeout(uint32_t timeout_mclks);
static uint16_t DecodeTimeout(uint16_t reg_val);
static uint32_t TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

static uint8_t GetSpadInfo(uint8_t* count, bool_t* type_is_aperture);
static void GetSequenceStepEnables(SequenceStepEnables_t* enables);
static void GetSequenceStepTimeouts(SequenceStepEnables_t const* enables, SequenceStepTimeouts_t* timeouts);
static uint8_t PerformSingleRefCalibration(uint8_t vhv_init_byte);
static void VL53L0X_HardReset(void);

Status_t Init_VL53L0X(uint8_t b_long_range)
{
    VL53L0X_HardReset();
    if (I2C_STATUS_OK != I2C_Manager_IsDeviceReady(VL530L0X_ADDRESS))
    {
        LogPrintf(LOG_ERROR, "Initializing sensor failed");
        return STATUS_TIMEOUT_ERROR;
    }

    return InitSensor(b_long_range);
}

static void VL53L0X_HardReset(void)
{
    HAL_GPIO_WritePin(VL53_XSHUT_GPIO_Port, VL53_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(VL53_XSHUT_GPIO_Port, VL53_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

static Status_t InitSensor(uint8_t io_2v8)
{
    Status_t status = STATUS_OK;
    uint8_t regVal  = 0u;

    status = ReadRegister(IDENTIFICATION_MODEL_ID, &regVal);
    if (STATUS_OK == status && regVal != 0xEE)
    {
        return STATUS_INVALID_READING;
    }

    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io_2v8)
    {
        ReadRegister(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &regVal);
        WriteRegister(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, regVal | 0x01); // set bit 0
    }

    // "Set I2C standard mode"
    WriteRegister(0x88, 0x00);

    WriteRegister(0x80, 0x01);
    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);
    ReadRegister(0x91, &stop_variable);
    WriteRegister(0x00, 0x01);
    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    ReadRegister(MSRC_CONFIG_CONTROL, &regVal);
    WriteRegister(MSRC_CONFIG_CONTROL, regVal | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    WriteRegister16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0.25 * (1 << 7));

    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    if (!GetSpadInfo(&spad_count, &spad_type_is_aperture))
    {
        return STATUS_ERROR;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    WriteRegister(0xFF, 0x01);
    WriteRegister(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    WriteRegister(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    WriteRegister(0xFF, 0x00);
    WriteRegister(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled        = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    WriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x09, 0x00);
    WriteRegister(0x10, 0x00);
    WriteRegister(0x11, 0x00);

    WriteRegister(0x24, 0x01);
    WriteRegister(0x25, 0xFF);
    WriteRegister(0x75, 0x00);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x4E, 0x2C);
    WriteRegister(0x48, 0x00);
    WriteRegister(0x30, 0x20);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x30, 0x09);
    WriteRegister(0x54, 0x00);
    WriteRegister(0x31, 0x04);
    WriteRegister(0x32, 0x03);
    WriteRegister(0x40, 0x83);
    WriteRegister(0x46, 0x25);
    WriteRegister(0x60, 0x00);
    WriteRegister(0x27, 0x00);
    WriteRegister(0x50, 0x06);
    WriteRegister(0x51, 0x00);
    WriteRegister(0x52, 0x96);
    WriteRegister(0x56, 0x08);
    WriteRegister(0x57, 0x30);
    WriteRegister(0x61, 0x00);
    WriteRegister(0x62, 0x00);
    WriteRegister(0x64, 0x00);
    WriteRegister(0x65, 0x00);
    WriteRegister(0x66, 0xA0);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x22, 0x32);
    WriteRegister(0x47, 0x14);
    WriteRegister(0x49, 0xFF);
    WriteRegister(0x4A, 0x00);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x7A, 0x0A);
    WriteRegister(0x7B, 0x00);
    WriteRegister(0x78, 0x21);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x23, 0x34);
    WriteRegister(0x42, 0x00);
    WriteRegister(0x44, 0xFF);
    WriteRegister(0x45, 0x26);
    WriteRegister(0x46, 0x05);
    WriteRegister(0x40, 0x40);
    WriteRegister(0x0E, 0x06);
    WriteRegister(0x20, 0x1A);
    WriteRegister(0x43, 0x40);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x34, 0x03);
    WriteRegister(0x35, 0x44);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x31, 0x04);
    WriteRegister(0x4B, 0x09);
    WriteRegister(0x4C, 0x05);
    WriteRegister(0x4D, 0x04);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x44, 0x00);
    WriteRegister(0x45, 0x20);
    WriteRegister(0x47, 0x08);
    WriteRegister(0x48, 0x28);
    WriteRegister(0x67, 0x00);
    WriteRegister(0x70, 0x04);
    WriteRegister(0x71, 0x01);
    WriteRegister(0x72, 0xFE);
    WriteRegister(0x76, 0x00);
    WriteRegister(0x77, 0x00);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x0D, 0x01);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x01);
    WriteRegister(0x01, 0xF8);

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x8E, 0x01);
    WriteRegister(0x00, 0x01);
    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    WriteRegister(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    ReadRegister(GPIO_HV_MUX_ACTIVE_HIGH, &regVal);
    WriteRegister(GPIO_HV_MUX_ACTIVE_HIGH, regVal & ~0x10); // active low
    WriteRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    measurement_timing_budget_us = GetMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    SetMeasurementTimingBudget(measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!PerformSingleRefCalibration(0x40))
    {
        return STATUS_ERROR;
    }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!PerformSingleRefCalibration(0x00))
    {
        return STATUS_ERROR;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return STATUS_OK;
}

static Status_t WriteRegister(uint8_t reg, uint8_t value)
{
    if (I2C_STATUS_OK != I2C_Manager_Write(VL530L0X_ADDRESS, reg, &value, 1))
    {
        LogPrintf(LOG_ERROR, "ReadRegister failed");
        return STATUS_I2C_ERROR;
    }
    return STATUS_OK;
}

static Status_t WriteRegister16Bit(uint8_t reg, uint16_t value)
{
    uint8_t writeVal[2];
    writeVal[0] = (uint8_t)(value >> 8);
    writeVal[1] = (uint8_t)(value & 0xFF);
    if (I2C_STATUS_OK != I2C_Manager_Write(VL530L0X_ADDRESS, reg, writeVal, 2))
    {
        LogPrintf(LOG_ERROR, "ReadRegister failed");
        return STATUS_I2C_ERROR;
    }
    return STATUS_OK;
}

static Status_t WriteRegister32Bit(uint8_t reg, uint32_t value)
{
    uint8_t writeVal[4];
    writeVal[0] = (uint8_t)(value >> 24);
    writeVal[1] = (uint8_t)(value >> 16);
    writeVal[2] = (uint8_t)(value >> 8);
    writeVal[3] = (uint8_t)(value & 0xFF);
    if (I2C_STATUS_OK != I2C_Manager_Write(VL530L0X_ADDRESS, reg, writeVal, 4))
    {
        LogPrintf(LOG_ERROR, "ReadRegister failed");
        return STATUS_I2C_ERROR;
    }
    return STATUS_OK;
}

static Status_t ReadRegister(uint8_t reg, uint8_t* data)
{
    if (I2C_STATUS_OK != I2C_Manager_Read(VL530L0X_ADDRESS, reg, data, 1))
    {
        LogPrintf(LOG_ERROR, "ReadRegister failed");
        return STATUS_I2C_ERROR;
    }
    return STATUS_OK;
}

static Status_t ReadRegister16Bit(uint8_t reg, uint16_t* data)
{
    uint8_t bytes[2];

    if (I2C_STATUS_OK != I2C_Manager_Read(VL530L0X_ADDRESS, reg, bytes, 2))
    {
        LogPrintf(LOG_ERROR, "Reading Data failed");
        return STATUS_I2C_ERROR;
    }

    *data = ((uint16_t)bytes[0] << 8) | bytes[1];
    return STATUS_OK;
}

static Status_t WriteMulti(uint8_t reg, uint8_t* src, uint8_t count)
{
    if (I2C_STATUS_OK != I2C_Manager_Write(VL530L0X_ADDRESS, reg, src, count))
    {
        LogPrintf(LOG_ERROR, "Writing Data failed");
        return STATUS_I2C_ERROR;
    }

    return STATUS_OK;
}

static Status_t ReadMulti(uint8_t reg, uint8_t* dst, uint8_t count)
{
    if (I2C_STATUS_OK != I2C_Manager_Read(VL530L0X_ADDRESS, reg, dst, count))
    {
        LogPrintf(LOG_ERROR, "Writing Data failed");
        return STATUS_I2C_ERROR;
    }

    return STATUS_OK;
}

// Get the return signal rate limit check value in MCPS
float GetSignalRateLimit()
{
    uint16_t regVal = 0u;
    ReadRegister16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &regVal);
    return (float)regVal / (1 << 7);
}

static uint8_t SetMeasurementTimingBudget(uint32_t budget_us)
{
    SequenceStepEnables_t enables;
    SequenceStepTimeouts_t timeouts;

    uint16_t const StartOverhead      = 1910;
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    GetSequenceStepEnables(&enables);
    GetSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us)
        {
            // "Requested timeout too big."
            return FALSE;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint32_t final_range_timeout_mclks = TimeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        WriteRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, EncodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return TRUE;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
static uint32_t GetMeasurementTimingBudget()
{
    SequenceStepEnables_t enables;
    SequenceStepTimeouts_t timeouts;

    uint16_t const StartOverhead      = 1910;
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    GetSequenceStepEnables(&enables);
    GetSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
uint8_t SetVcselPulsePeriod(vcselPeriodType_t type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables_t enables;
    SequenceStepTimeouts_t timeouts;

    GetSequenceStepEnables(&enables);
    GetSequenceStepTimeouts(&enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type == VcselPeriodPreRange)
    {
        // "Set phase check limits"
        switch (period_pclks)
        {
        case 12:
            WriteRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;

        case 14:
            WriteRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;

        case 16:
            WriteRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;

        case 18:
            WriteRegister(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;

        default:
            // invalid period
            return FALSE;
        }
        WriteRegister(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        WriteRegister(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks =
            TimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        WriteRegister16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, EncodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks = TimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        WriteRegister(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    }
    else if (type == VcselPeriodFinalRange)
    {
        switch (period_pclks)
        {
        case 8:
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            WriteRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            WriteRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            WriteRegister(0xFF, 0x01);
            WriteRegister(ALGO_PHASECAL_LIM, 0x30);
            WriteRegister(0xFF, 0x00);
            break;

        case 10:
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            WriteRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            WriteRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            WriteRegister(0xFF, 0x01);
            WriteRegister(ALGO_PHASECAL_LIM, 0x20);
            WriteRegister(0xFF, 0x00);
            break;

        case 12:
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            WriteRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            WriteRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            WriteRegister(0xFF, 0x01);
            WriteRegister(ALGO_PHASECAL_LIM, 0x20);
            WriteRegister(0xFF, 0x00);
            break;

        case 14:
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            WriteRegister(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            WriteRegister(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            WriteRegister(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            WriteRegister(0xFF, 0x01);
            WriteRegister(ALGO_PHASECAL_LIM, 0x20);
            WriteRegister(0xFF, 0x00);
            break;

        default:
            // invalid period
            return FALSE;
        }

        // apply new VCSEL period
        WriteRegister(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
            TimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range)
        {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        WriteRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, EncodeTimeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
    }
    else
    {
        // invalid type
        return FALSE;
    }

    // "Finally, the timing budget must be re-applied"

    SetMeasurementTimingBudget(measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = 0u;
    ReadRegister(SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    WriteRegister(SYSTEM_SEQUENCE_CONFIG, 0x02);
    PerformSingleRefCalibration(0x0);
    WriteRegister(SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return TRUE;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
static uint8_t getVcselPulsePeriod(vcselPeriodType_t type)
{
    uint8_t regVal = 0u;
    if (type == VcselPeriodPreRange)
    {
        ReadRegister(PRE_RANGE_CONFIG_VCSEL_PERIOD, &regVal);
        return decodeVcselPeriod(regVal);
    }
    else if (type == VcselPeriodFinalRange)
    {
        ReadRegister(FINAL_RANGE_CONFIG_VCSEL_PERIOD, &regVal);
        return decodeVcselPeriod(regVal);
    }
    else
    {
        return 255;
    }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(uint32_t period_ms)
{
    WriteRegister(0x80, 0x01);
    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);
    WriteRegister(0x91, stop_variable);
    WriteRegister(0x00, 0x01);
    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x00);

    if (period_ms != 0)
    {
        uint16_t regVal = 0u;
        ReadRegister16Bit(OSC_CALIBRATE_VAL, &regVal);
        uint16_t osc_calibrate_val = regVal;

        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }

        WriteRegister32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        WriteRegister(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else
    {
        // continuous back-to-back mode
        WriteRegister(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stopContinuous()
{
    WriteRegister(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);
    WriteRegister(0x91, 0x00);
    WriteRegister(0x00, 0x01);
    WriteRegister(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t readRangeContinuousMillimeters()
{
    startTimeout();
    uint8_t regVal = 0u;
    while ((STATUS_OK == ReadRegister(RESULT_INTERRUPT_STATUS, &regVal)) && ((regVal & 0x07) == 0))
    {
        if (checkTimeoutExpired())
        {
            did_timeout = TRUE;
            return 65535;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = 0u;
    ReadRegister16Bit(RESULT_RANGE_STATUS + 10, &range);

    WriteRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t readRangeSingleMillimeters()
{
    WriteRegister(0x80, 0x01);
    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);
    WriteRegister(0x91, stop_variable);
    WriteRegister(0x00, 0x01);
    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x00);

    WriteRegister(SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    uint8_t regVal = 0u;
    while (STATUS_OK == ReadRegister(SYSRANGE_START, &regVal) && (regVal & 0x01))
    {
        if (checkTimeoutExpired())
        {
            did_timeout = TRUE;
            return 65535;
        }
    }

    return readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
uint8_t timeoutOccurred()
{
    uint8_t tmp = did_timeout;
    did_timeout = FALSE;
    return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
static uint8_t GetSpadInfo(uint8_t* count, bool_t* type_is_aperture)
{
    uint8_t tmp    = 0u;
    uint8_t regVal = 0u;
    WriteRegister(0x80, 0x01);
    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x00);

    WriteRegister(0xFF, 0x06);
    ReadRegister(0x83, &regVal);
    WriteRegister(0x83, regVal | 0x04);
    WriteRegister(0xFF, 0x07);
    WriteRegister(0x81, 0x01);

    WriteRegister(0x80, 0x01);

    WriteRegister(0x94, 0x6b);
    WriteRegister(0x83, 0x00);
    startTimeout();
    while ((STATUS_OK == ReadRegister(0x83, &regVal)) && (regVal == 0x00))
    {
        if (checkTimeoutExpired())
        {
            return FALSE;
        }
    }
    WriteRegister(0x83, 0x01);
    ReadRegister(0x92, &tmp);

    *count            = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    WriteRegister(0x81, 0x00);
    WriteRegister(0xFF, 0x06);
    ReadRegister(0x83, &regVal);
    WriteRegister(0x83, regVal & ~0x04);
    WriteRegister(0xFF, 0x01);
    WriteRegister(0x00, 0x01);

    WriteRegister(0xFF, 0x00);
    WriteRegister(0x80, 0x00);

    return TRUE;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static void GetSequenceStepEnables(SequenceStepEnables_t* enables)
{
    uint8_t sequence_config = 0u;
    ReadRegister(SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    enables->tcc         = (sequence_config >> 4) & 0x1;
    enables->dss         = (sequence_config >> 3) & 0x1;
    enables->msrc        = (sequence_config >> 2) & 0x1;
    enables->pre_range   = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
static void GetSequenceStepTimeouts(SequenceStepEnables_t const* enables, SequenceStepTimeouts_t* timeouts)
{
    uint8_t regVal       = 0u;
    uint16_t regVal16Bit = 0u;

    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

    ReadRegister(MSRC_CONFIG_TIMEOUT_MACROP, &regVal);
    timeouts->msrc_dss_tcc_mclks = regVal + 1;
    timeouts->msrc_dss_tcc_us    = TimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    ReadRegister16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &regVal16Bit);
    timeouts->pre_range_mclks = DecodeTimeout(regVal16Bit);
    timeouts->pre_range_us    = TimeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

    ReadRegister16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &regVal16Bit);
    timeouts->final_range_mclks = DecodeTimeout(regVal16Bit);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = TimeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t DecodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
static uint16_t EncodeTimeout(uint32_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

static uint32_t TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
static uint8_t PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
    WriteRegister(SYSRANGE_START, 0x01 | vhv_init_byte);

    startTimeout();
    uint8_t regVal = 0u;
    while (STATUS_OK == ReadRegister(RESULT_INTERRUPT_STATUS, &regVal) && ((regVal & 0x07) == 0))
    {
        if (checkTimeoutExpired())
        {
            return FALSE;
        }
    }

    WriteRegister(SYSTEM_INTERRUPT_CLEAR, 0x01);

    WriteRegister(SYSRANGE_START, 0x00);

    return TRUE;
}
