/*
 * bme.c
 * Driver for BME280 Sensor
 * This code is based on
 * https://github.com/boschsensortec/BME280_SensorAPI
 *
 * Ported to STM32 Cube HAL by Wojciech Niewiadomski
 *  Created on: Jan 29, 2026
 *
 */

#include "bme.h"
#include "i2c_manager.h"
#include "logger.h"
#include "utils.h"

/*! @name Bit shift macros */
#define BME280_12_BIT_SHIFT UINT8_C(12)
#define BME280_8_BIT_SHIFT UINT8_C(8)
#define BME280_4_BIT_SHIFT UINT8_C(4)

/*! @name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define MODE_SLEEP 0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

// Register names:
#define BME280_CTRL_HUMIDITY_REG 0xF2 // Ctrl Humidity Reg
#define BME280_STAT_REG 0xF3          // Status Reg
#define BME280_REG_PWR_CTRL 0xF4      // Ctrl Measure Reg
#define BME280_CONFIG_REG 0xF5        // Configuration Reg

#define BME280_PRESSURE_MSB_REG 0xF7    // Pressure MSB
#define BME280_TEMPERATURE_MSB_REG 0xFA // Temperature MSB
#define BME280_HUMIDITY_MSB_REG 0xFD    // Humidity MSB
#define BME280_DATA_REG 0xF7

#define BME280_REG_TEMP_PRESS_CALIB_DATA 0x88
#define BME280_LEN_TEMP_PRESS_CALIB_DATA 26

#define BME280_REG_HUMIDITY_CALIB_DATA 0xE1
#define BME280_LEN_HUMIDITY_CALIB_DATA 7

typedef struct
{
    uint8_t temperatureOversampling;
    uint8_t pressureOversampling;
    uint8_t humidityOversampling;
    uint8_t filter;
    uint8_t standbyTime;
} BME280_SensorSettings;

typedef struct BME280_CalibrationData_t
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
    int32_t t_fine;
} BME280_CalibrationData_t;

typedef struct BME280_UncompData_t
{
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
} BME280_UncompData_t;

static BME280_CalibrationData_t calibration;
static Status_t status = STATUS_OK;

static uint8_t ReadRegister(unsigned char reg);
static void ReadMulti(unsigned char addr, uint8_t* dst, uint32_t size);
static void WriteRegister(unsigned char reg, unsigned char data);
static float CompensateHumidity(const BME280_UncompData_t* uncomp_data);
static float CompensatePressure(const BME280_UncompData_t* uncomp_data);
static float CompensateTemperature(const BME280_UncompData_t* uncomp_data);
static void ParseSensorData(const uint8_t* reg_data, BME280_UncompData_t* uncomp_data);
static void GetHumidityCalibrationData();
static void GetTemperatureAndPressureCalibrationData();
static void SetMode(unsigned char mode);

static uint8_t ReadRegister(unsigned char reg)
{
    uint8_t buffer;

    if (I2C_STATUS_OK == I2C_Manager_Read(BME280_ADDRESS, reg, &buffer, 1))
    {
        status = STATUS_OK;
    }
    else
    {
        status = I2C_ERROR;
    }

    return buffer;
}

static void ReadMulti(unsigned char addr, uint8_t* dst, uint32_t size)
{
    if (I2C_STATUS_OK == I2C_Manager_Read(BME280_ADDRESS, addr, dst, size))
    {
        status = STATUS_OK;
    }
    else
    {
        status = I2C_ERROR;
    }
}

static void WriteRegister(unsigned char reg, unsigned char data)
{
    if (I2C_STATUS_OK == I2C_Manager_Write(BME280_ADDRESS, reg, &data, 1))
    {
        status = STATUS_OK;
    }
    else
    {
        status = I2C_ERROR;
    }
}

static void GetTemperatureAndPressureCalibrationData()
{
    uint8_t reg_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

    ReadMulti(BME280_REG_TEMP_PRESS_CALIB_DATA, reg_data, BME280_LEN_TEMP_PRESS_CALIB_DATA);
    if (STATUS_OK != status)
    {
        LogPrintf("[error][BME280] Temperature and pressure calibration data reading failed\n");
    }

    calibration.dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calibration.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calibration.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calibration.dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calibration.dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calibration.dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calibration.dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calibration.dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calibration.dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calibration.dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calibration.dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calibration.dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
}

static void GetHumidityCalibrationData()
{
    uint8_t reg_data[BME280_LEN_HUMIDITY_CALIB_DATA] = { 0 };

    ReadMulti(BME280_REG_HUMIDITY_CALIB_DATA, reg_data, BME280_LEN_HUMIDITY_CALIB_DATA);
    if (STATUS_OK != status)
    {
        LogPrintf("[error][BME280] Humidity calibration data reading failed\n");
    }

    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calibration.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calibration.dig_h3 = reg_data[2];
    dig_h4_msb         = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb         = (int16_t)(reg_data[4] & 0x0F);
    calibration.dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb         = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb         = (int16_t)(reg_data[4] >> 4);
    calibration.dig_h5 = dig_h5_msb | dig_h5_lsb;
    calibration.dig_h6 = (int8_t)reg_data[6];
}

void BME280_Init()
{

    if (I2C_STATUS_OK != I2C_Manager_IsDeviceReady(BME280_ADDRESS))
    {
        status = TIMEOUT_ERROR;
        LogPrintf("[error][BME280] Initializing sensor failed\n");
        return;
    }

    GetTemperatureAndPressureCalibrationData();
    GetHumidityCalibrationData();

    SetMode(MODE_NORMAL);
}

static void SetMode(unsigned char mode)
{
    if (mode > MODE_NORMAL)
        mode = 0; // Error check. Default to sleep mode

    uint8_t controlData = ReadRegister(BME280_REG_PWR_CTRL);
    if (STATUS_OK == status)
    {
        controlData &= ~((1 << 1) | (1 << 0)); // Clear the mode[1:0] bits
        controlData |= mode;                   // Set
        WriteRegister(BME280_REG_PWR_CTRL, controlData);
    }
}

static float CompensateTemperature(const BME280_UncompData_t* uncomp_data)
{
    float var1;
    float var2;
    float temperature;
    float temperature_min = -40;
    float temperature_max = 85;

    var1               = (((float)uncomp_data->temperature) / 16384.0 - ((float)calibration.dig_t1) / 1024.0);
    var1               = var1 * ((float)calibration.dig_t2);
    var2               = (((float)uncomp_data->temperature) / 131072.0 - ((float)calibration.dig_t1) / 8192.0);
    var2               = (var2 * var2) * ((float)calibration.dig_t3);
    calibration.t_fine = (int32_t)(var1 + var2);
    temperature        = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

static float CompensatePressure(const BME280_UncompData_t* uncomp_data)
{
    float var1;
    float var2;
    float var3;
    float pressure;
    float pressure_min = 30000.0;
    float pressure_max = 110000.0;

    var1 = ((float)calibration.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float)calibration.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((float)calibration.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((float)calibration.dig_p4) * 65536.0);
    var3 = ((float)calibration.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((float)calibration.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((float)calibration.dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (float)uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1     = ((float)calibration.dig_p9) * pressure * pressure / 2147483648.0;
        var2     = pressure * ((float)calibration.dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((float)calibration.dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

static float CompensateHumidity(const BME280_UncompData_t* uncomp_data)
{
    float humidity;
    float humidity_min = 0.0;
    float humidity_max = 100.0;
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    float var6;

    var1     = ((float)calibration.t_fine) - 76800.0;
    var2     = (((float)calibration.dig_h4) * 64.0 + (((float)calibration.dig_h5) / 16384.0) * var1);
    var3     = uncomp_data->humidity - var2;
    var4     = ((float)calibration.dig_h2) / 65536.0;
    var5     = (1.0 + (((float)calibration.dig_h3) / 67108864.0) * var1);
    var6     = 1.0 + (((float)calibration.dig_h6) / 67108864.0) * var1 * var5;
    var6     = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((float)calibration.dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

static void ParseSensorData(const uint8_t* reg_data, BME280_UncompData_t* uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb              = (uint32_t)reg_data[0] << BME280_12_BIT_SHIFT;
    data_lsb              = (uint32_t)reg_data[1] << BME280_4_BIT_SHIFT;
    data_xlsb             = (uint32_t)reg_data[2] >> BME280_4_BIT_SHIFT;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb                 = (uint32_t)reg_data[3] << BME280_12_BIT_SHIFT;
    data_lsb                 = (uint32_t)reg_data[4] << BME280_4_BIT_SHIFT;
    data_xlsb                = (uint32_t)reg_data[5] >> BME280_4_BIT_SHIFT;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb              = (uint32_t)reg_data[6] << BME280_8_BIT_SHIFT;
    data_lsb              = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

BME280_Data_t BME280_ReadData()
{
    BME280_UncompData_t uncompData = { 0 };
    BME280_Data_t compData         = { 0 };
    uint8_t regData[8]             = { 0 };
    if (I2C_STATUS_OK != I2C_Manager_Read(BME280_ADDRESS, BME280_DATA_REG, regData, 8))
    {
        LogPrintf("[error][BME280] Error reading sensor data\n");
    }

    ParseSensorData(regData, &uncompData);
    compData.humidity    = CompensateHumidity(&uncompData);
    compData.pressure    = CompensatePressure(&uncompData);
    compData.temperature = CompensateTemperature(&uncompData);

    compData.pressure /= 100; // Pa -> hPa

    return compData;
}

float BME280_ReadTemperature(void)
{
    BME280_Data_t readData = BME280_ReadData();
    return readData.temperature;
}

float BME280_ReadPressure(void)
{
    BME280_Data_t readData = BME280_ReadData();
    return readData.pressure;
}

float BME280_ReadHumidity(void)
{
    BME280_Data_t readData = BME280_ReadData();
    return readData.humidity;
}
