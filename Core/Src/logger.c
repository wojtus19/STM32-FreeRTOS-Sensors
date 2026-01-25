/*
 * logger.c
 *
 *  Created on: Jan 25, 2026
 *      Author: Wojciech Niewiadomski
 */

#include "logger.h"
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "stream_buffer.h"
#include "task.h"
#include <stdio.h>

#define LOGGER_BUFFER_SIZE 256
#define LOGGER_STREAM_SIZE 1024
#define LOGGER_USE_DMA 1

static StreamBufferHandle_t logStream;
extern UART_HandleTypeDef huart2;
extern TaskHandle_t logger_task_handle;
// static uint8_t txBuffer[LOGGER_BUFFER_SIZE];

void LoggerInit(void)
{
    logStream = xStreamBufferCreate(LOGGER_STREAM_SIZE, 1);
}

void LogPrintf(const char* fmt, ...)
{
    static char buffer[LOGGER_BUFFER_SIZE];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        xStreamBufferSend(logStream, buffer, len, 0);
    }
}

void LoggerTask(void* argument)
{
    static uint8_t txBuffer[LOGGER_BUFFER_SIZE];
    size_t len = 0;

    for (;;)
    {
        len = xStreamBufferReceive(logStream, txBuffer, sizeof(txBuffer), portMAX_DELAY);

        if (len > 0)
        {
#if LOGGER_USE_DMA
            // wait 1ms for DMA to free UART
            while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            HAL_UART_Transmit_DMA(&huart2, txBuffer, len);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
            HAL_UART_Transmit(&huart2, txBuffer, len, HAL_MAX_DELAY);
#endif
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(
            logger_task_handle,
            &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
