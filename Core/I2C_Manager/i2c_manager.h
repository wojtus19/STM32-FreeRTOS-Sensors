#pragma once

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"

typedef enum I2C_Status_t
{
    I2C_STATUS_OK,
    I2C_STATUS_ERROR,
    I2C_TIMEOUT
} I2C_Status_t;

void I2C_ManagerTask(void* argument);
void I2C_ManagerInit(I2C_HandleTypeDef* p_i2c_handler);
I2C_Status_t I2C_Manager_Write(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize);
I2C_Status_t I2C_Manager_Read(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t dataSize);
I2C_Status_t I2C_Manager_Transmit(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize);
I2C_Status_t I2C_Manager_Receive(uint16_t DevAddress, uint8_t* pData, uint16_t dataSize);
I2C_Status_t I2C_Manager_IsDeviceReady(uint16_t DevAddress);
