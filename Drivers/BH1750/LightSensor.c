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

#define BH1750_CLY_HRM 0x10 // continous mode
#define BH1750_OT_HRM 0x20  // one time mode

static Status_t status     = STATUS_OK;
static uint8_t initialized = FALSE;

static void WriteCommand(uint8_t command)
{
    if (I2C_STATUS_OK == I2C_Manager_Transmit(BH1750_ADDRESS, &command, 1))
    {
        status = STATUS_OK;
    }
    else
    {
        LogPrintf("[error][BH1750] Writing command failed\n");
        status = STATUS_I2C_ERROR;
    }
}

static uint16_t ReadData()
{

    uint8_t bytes[2] = { 0 };

    if (I2C_STATUS_OK == I2C_Manager_Receive(BH1750_ADDRESS, bytes, 2))
    {
        status = STATUS_OK;
    }
    else
    {
        LogPrintf("[error][BH1750] Reading Data\n");
        status = STATUS_I2C_ERROR;
    }

    return ((uint16_t)bytes[0] << 8) | bytes[1];
}

void BH1750_Init()
{
    if (I2C_STATUS_OK != I2C_Manager_IsDeviceReady(BH1750_ADDRESS))
    {
        status = STATUS_TIMEOUT_ERROR;
        LogPrintf("[error][BH1750] Initializing sensor failed\n");
        return;
    }
    initialized = TRUE;
    status      = STATUS_OK;

    WriteCommand(BH1750_POWER_ON);
    WriteCommand(BH1750_CLY_HRM);
}

float BH1750_ReadLux()
{
    if (!initialized)
    {
        return 0.0;
    }
    uint16_t raw = ReadData();
    if (STATUS_OK != status)
    {
        return 0.0;
    }

    float lux = raw / 1.2f;
    return lux;
}
