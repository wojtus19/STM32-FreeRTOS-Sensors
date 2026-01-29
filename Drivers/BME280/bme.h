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

typedef struct BME280_Data_t
{
    float pressure;
    float temperature;
    float humidity;
} BME280_Data_t;

void BME280_Init();
float BME280_ReadTemperature(void);
float BME280_ReadPressure(void);
float BME280_ReadHumidity(void);
BME280_Data_t BME280_ReadData();

#endif /* BME280_BME_H_ */
