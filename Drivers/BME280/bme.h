/*
 * bme.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Wojciech Niewiadomski
 */

#ifndef BME280_BME_H_
#define BME280_BME_H_

#include "stm32f4xx_hal.h"
#include "typedef.h"
#include <stdfix.h>
#include <stdint.h>
#include <stdio.h>

#define BME280_ADDRESS (0x76 << 1)

typedef enum
{
    BME280_NO_OVERSAMPLING  = 0x00,
    BME280_OVERSAMPLING_1X  = 0x01,
    BME280_OVERSAMPLING_2X  = 0x02,
    BME280_OVERSAMPLING_4X  = 0x03,
    BME280_OVERSAMPLING_8X  = 0x04,
    BME280_OVERSAMPLING_16X = 0x05,
    BME280_OVERSAMPLING_COUNT
} OversamplingFactor_t;

typedef enum
{
    BME280_FILTER_COEFF_OFF = 0x00,
    BME280_FILTER_COEFF_2   = 0x01,
    BME280_FILTER_COEFF_4   = 0x02,
    BME280_FILTER_COEFF_8   = 0x03,
    BME280_FILTER_COEFF_16  = 0x04,
    BME280_FILTER_COEFF_COUNT
} FilterCoeff_t;

typedef enum
{
    BME280_STANDBY_TIME_0_5_MS  = 0x00,
    BME280_STANDBY_TIME_62_5_MS = 0x01,
    BME280_STANDBY_TIME_125_MS  = 0x02,
    BME280_STANDBY_TIME_250_MS  = 0x03,
    BME280_STANDBY_TIME_500_MS  = 0x04,
    BME280_STANDBY_TIME_1000_MS = 0x05,
    BME280_STANDBY_TIME_10_MS   = 0x06,
    BME280_STANDBY_TIME_20_MS   = 0x07,
    BME280_STANDBY_TIME_COUNT
} StandbyFactor_t;

typedef struct BME280_Data_t
{
    float pressure;
    float temperature;
    float humidity;
} BME280_Data_t;

typedef struct BME280_Settings_t
{
    /*! Pressure oversampling */
    OversamplingFactor_t pressureOversampling;

    /*! Temperature oversampling */
    OversamplingFactor_t temperatureOversampling;

    /*! Humidity oversampling */
    OversamplingFactor_t humidityOversampling;

    /*! Filter coefficient */
    FilterCoeff_t filter;

    /*! Standby time */
    StandbyFactor_t standbyTime;
} BME280_Settings_t;

Status_t BME280_Init();
float BME280_ReadTemperature(void);
float BME280_ReadPressure(void);
float BME280_ReadHumidity(void);
BME280_Data_t BME280_ReadData();
Status_t BME280_SetSettings(BME280_Settings_t settings);

#endif /* BME280_BME_H_ */
