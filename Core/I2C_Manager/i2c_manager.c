/*
 * i2c_manager.c
 *
 *  Created on: Jan 28, 2026
 *      Author: Wojciech Niewiadomski
 */

#include "i2c_manager.h"
#include "FreeRTOS.h"
#include "logger.h"
#include "queue.h"
#include "semphr.h"

#define I2C_QUEUE_DEPTH 4
#define I2C_TIMEOUT 1000

static QueueHandle_t i2cQueue;
static I2C_HandleTypeDef hi2c1;

extern TaskHandle_t I2C_ManagerTaskHandle;

static I2C_Status_t I2C_WriteReg(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize);
static I2C_Status_t I2C_ReadReg(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize);
static I2C_Status_t I2C_Transmit(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize);
static I2C_Status_t I2C_Receive(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize);
static I2C_Status_t I2C_IsDeviceReady(uint16_t DevAddress);

typedef enum I2C_Cmd_t
{
    I2C_READ_REG,
    I2C_WRITE_REG,
    I2C_RECEIVE,
    I2C_TRANSMIT,
    I2C_IS_DEVICE_READY
} I2C_Cmd_t;

typedef struct I2C_Request_t
{
    I2C_Cmd_t requestType;
    uint16_t devAddress;
    uint16_t memAddress;
    uint16_t bufferSize;
    uint8_t* pBuffer;
    SemaphoreHandle_t doneSemaphore;
    I2C_Status_t status;
} I2C_Request_t;

void I2C_ManagerInit(I2C_HandleTypeDef* p_i2c_handler)
{
    hi2c1    = *p_i2c_handler;
    i2cQueue = xQueueCreate(I2C_QUEUE_DEPTH, sizeof(I2C_Request_t));
}

void I2C_ManagerTask(void* argument)
{
    I2C_Request_t req;
    req.status = I2C_STATUS_OK;
    for (;;)
    {
        xQueueReceive(i2cQueue, &req, portMAX_DELAY);
        switch (req.requestType)
        {
        case I2C_READ_REG:
            req.status = I2C_ReadReg(req.devAddress, req.memAddress, req.pBuffer, req.bufferSize);
            break;
        case I2C_WRITE_REG:
            req.status = I2C_WriteReg(req.devAddress, req.memAddress, req.pBuffer, req.bufferSize);
            break;
        case I2C_TRANSMIT:
            req.status = I2C_Transmit(req.devAddress, req.pBuffer, req.bufferSize);
            break;
        case I2C_RECEIVE:
            req.status = I2C_Receive(req.devAddress, req.pBuffer, req.bufferSize);
            break;
        case I2C_IS_DEVICE_READY:
            req.status = I2C_IsDeviceReady(req.devAddress);
        default:
            break;
        }
        if (I2C_STATUS_OK != req.status)
        {
            LogPrintf(LOG_ERROR, "I2C status: %d", req.status);
        }
        xSemaphoreGive(req.doneSemaphore);
    }
}

I2C_Status_t I2C_Manager_Write(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Request_t req;
    req.requestType   = I2C_WRITE_REG;
    req.devAddress    = DevAddress;
    req.memAddress    = MemAddress;
    req.bufferSize    = dataSize;
    req.pBuffer       = pData;
    req.status        = I2C_STATUS_OK;
    req.doneSemaphore = xSemaphoreCreateBinary();

    if (xQueueSend(i2cQueue, &req, portMAX_DELAY) != pdPASS)
    {
        LogPrintf(LOG_ERROR, "I2C Queue timeout");
        req.status = I2C_STATUS_ERROR;
    }
    xSemaphoreTake(req.doneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(req.doneSemaphore);
    return req.status;
}

I2C_Status_t I2C_Manager_Read(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Request_t req;
    req.requestType = I2C_READ_REG;
    req.devAddress  = DevAddress;
    req.memAddress  = MemAddress;
    req.bufferSize  = dataSize;
    req.pBuffer     = pData;
    req.status      = I2C_STATUS_OK;

    req.doneSemaphore = xSemaphoreCreateBinary();

    if (xQueueSend(i2cQueue, &req, portMAX_DELAY) != pdPASS)
    {
        LogPrintf(LOG_ERROR, "I2C Queue timeout");
        req.status = I2C_STATUS_ERROR;
    }
    xSemaphoreTake(req.doneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(req.doneSemaphore);
    return req.status;
}

I2C_Status_t I2C_Manager_IsDeviceReady(uint16_t DevAddress)
{
    I2C_Request_t req;
    req.requestType   = I2C_IS_DEVICE_READY;
    req.devAddress    = DevAddress;
    req.memAddress    = 0;    // not used
    req.bufferSize    = 0;    // not used
    req.pBuffer       = NULL; // not used
    req.status        = I2C_STATUS_OK;
    req.doneSemaphore = xSemaphoreCreateBinary();

    if (xQueueSend(i2cQueue, &req, portMAX_DELAY) != pdPASS)
    {
        LogPrintf(LOG_ERROR, "I2C Queue timeout");
        req.status = I2C_STATUS_ERROR;
    }
    xSemaphoreTake(req.doneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(req.doneSemaphore);
    return req.status;
}

I2C_Status_t I2C_Manager_Transmit(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Request_t req;
    req.requestType   = I2C_TRANSMIT;
    req.devAddress    = DevAddress;
    req.memAddress    = 0; // not used
    req.bufferSize    = dataSize;
    req.pBuffer       = pData;
    req.status        = I2C_STATUS_OK;
    req.doneSemaphore = xSemaphoreCreateBinary();

    if (xQueueSend(i2cQueue, &req, portMAX_DELAY) != pdPASS)
    {
        LogPrintf(LOG_ERROR, "I2C Queue timeout");
        req.status = I2C_STATUS_ERROR;
    }
    xSemaphoreTake(req.doneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(req.doneSemaphore);
    return req.status;
}

I2C_Status_t I2C_Manager_Receive(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Request_t req;
    req.requestType   = I2C_RECEIVE;
    req.devAddress    = DevAddress;
    req.memAddress    = 0; // not used
    req.bufferSize    = dataSize;
    req.pBuffer       = pData;
    req.status        = I2C_STATUS_OK;
    req.doneSemaphore = xSemaphoreCreateBinary();

    if (xQueueSend(i2cQueue, &req, portMAX_DELAY) != pdPASS)
    {
        LogPrintf(LOG_ERROR, "I2C Queue timeout");
        req.status = I2C_STATUS_ERROR;
    }
    xSemaphoreTake(req.doneSemaphore, portMAX_DELAY);
    vSemaphoreDelete(req.doneSemaphore);
    return req.status;
}

static I2C_Status_t I2C_IsDeviceReady(uint16_t DevAddress)
{
    I2C_Status_t status = I2C_STATUS_OK;
    if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, DevAddress, 1, I2C_TIMEOUT))
    {
        status = I2C_STATUS_OK;
    }
    else
    {
        status = I2C_STATUS_ERROR;
    }
    return status;
}

I2C_Status_t I2C_ReadReg(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Status_t status = I2C_STATUS_OK;
    if (HAL_OK == HAL_I2C_Mem_Read(&hi2c1, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, dataSize, I2C_TIMEOUT))
    {
        status = I2C_STATUS_OK;
    }
    else
    {
        status = I2C_STATUS_ERROR;
    }
    return status;
}

I2C_Status_t I2C_WriteReg(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Status_t status = I2C_STATUS_OK;
    if (HAL_OK == HAL_I2C_Mem_Write(&hi2c1, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, dataSize, I2C_TIMEOUT))
    {
        status = I2C_STATUS_OK;
    }
    else
    {
        status = I2C_STATUS_ERROR;
    }
    return status;
}

static I2C_Status_t I2C_Transmit(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Status_t status = I2C_STATUS_OK;
    if (HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, dataSize, I2C_TIMEOUT))
    {
        status = I2C_STATUS_OK;
    }
    else
    {
        status = I2C_STATUS_ERROR;
    }
    return status;
}

static I2C_Status_t I2C_Receive(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize)
{
    I2C_Status_t status = I2C_STATUS_OK;
    if (HAL_OK == HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, dataSize, I2C_TIMEOUT))
    {
        status = I2C_STATUS_OK;
    }
    else
    {
        status = I2C_STATUS_ERROR;
    }
    return status;
}