/*
 * LightSensor.c
 *
 *  Created on: Jan 29, 2026
 *      Author: Wojtus
 */

#include "LightSensor.h"
#include "i2c_manager.h"
#include "logger.h"
#include "typedef.h"

#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01

#define BH1750_CONTINUOUS_HRM 0x10 // continuous mode
#define BH1750_ONE_TIME_HRM 0x20   // one time mode

static Status_t WriteCommand(uint8_t command);
static Status_t ReadData(uint16_t* data);

static uint8_t initialized = FALSE;

static Status_t WriteCommand(uint8_t command)
{
    if (I2C_STATUS_OK != I2C_Manager_Transmit(BH1750_ADDRESS, &command, 1))
    {
        LogPrintf(LOG_ERROR, "Writing command failed");
        return STATUS_I2C_ERROR;
    }
    return STATUS_OK;
}

static Status_t ReadData(uint16_t* data)
{
    uint8_t bytes[2];

    if (I2C_STATUS_OK != I2C_Manager_Receive(BH1750_ADDRESS, bytes, 2))
    {
        LogPrintf(LOG_ERROR, "Reading Data");
        return STATUS_I2C_ERROR;
    }

    *data = ((uint16_t)bytes[0] << 8) | bytes[1];
    return STATUS_OK;
}

Status_t BH1750_Init()
{
    Status_t status = STATUS_OK;
    if (I2C_STATUS_OK != I2C_Manager_IsDeviceReady(BH1750_ADDRESS))
    {
        LogPrintf(LOG_ERROR, "Initializing sensor failed");
        return STATUS_TIMEOUT_ERROR;
    }
    initialized = TRUE;

    status = WriteCommand(BH1750_POWER_ON);
    if (STATUS_OK != status)
        return status;
    status = WriteCommand(BH1750_CONTINUOUS_HRM);

    return status;
}

Status_t BH1750_Deinit()
{
    initialized = FALSE;
    return WriteCommand(BH1750_POWER_DOWN);
}

Status_t BH1750_ReadLux(float* lux)
{
    if (NULL == lux)
    {
        return STATUS_INVALID_PARAMS;
    }

    Status_t status = STATUS_OK;
    if (!initialized)
    {
        return STATUS_INVALID_READING;
    }
    uint16_t raw = 0u;

    status = ReadData(&raw);
    if (STATUS_OK != status)
    {
        return STATUS_INVALID_READING;
    }

    *lux = raw / 1.2f;
    return STATUS_OK;
}
