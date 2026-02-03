#include "FreeRTOS.h"
#include "LCD_Screen/LCD_Screen.h"
#include "logger.h"
#include "stdint.h"
#include "stdio.h"
#include "task.h"

#ifdef DEBUG_LOGS_ENABLED
#define STATS_BUFFER_SIZE 512
#endif

#define MAX_TASKS 10
#define HIGHWATER_MARK_MIN 20

static TaskStatus_t taskStatusArray[MAX_TASKS];
static char statsBuffer[STATS_BUFFER_SIZE];
static uint32_t prevTotalTime = 0u;
static uint32_t prevIdleTime  = 0u;

extern TIM_HandleTypeDef htim2;

void ConfigureTimerForRunTimeStats(void)
{
    HAL_TIM_Base_Start(&htim2);
}

uint32_t GetRunTimeCounterValue(void)
{
    return TIM2->CNT;
}

void DiagnosticTask(void* argument)
{
    UBaseType_t taskCount = 0u;
    uint32_t totalRunTime = 0u;
    uint32_t idleRunTime  = 0u;
    uint32_t deltaTotal   = 0u;
    uint32_t deltaIdle    = 0u;
    uint32_t cpuUsage     = 0u;
    char cpuUsageStr[20];

    TaskHandle_t idleHandle = xTaskGetIdleTaskHandle();
    for (;;)
    {
        taskCount = uxTaskGetSystemState(taskStatusArray, MAX_TASKS, &totalRunTime);

        idleRunTime = 0;
        for (UBaseType_t i = 0; i < taskCount; i++)
        {
            if (taskStatusArray[i].xHandle == idleHandle)
            {
                idleRunTime = taskStatusArray[i].ulRunTimeCounter;
                break;
            }
        }

        deltaTotal = totalRunTime - prevTotalTime;
        deltaIdle  = idleRunTime - prevIdleTime;

        if (deltaTotal > 0)
        {
            cpuUsage = 100 - ((deltaIdle * 100) / deltaTotal);
            if (cpuUsage > 100)
            {
                cpuUsage = 100;
            }
        }
        else
        {
            cpuUsage = 0;
        }

        prevTotalTime = totalRunTime;
        prevIdleTime  = idleRunTime;

        snprintf(cpuUsageStr, sizeof(cpuUsageStr), "CPU Usage: %lu%%", cpuUsage);
        LCD_FillRect(0, 220, LCD_WIDTH, LCD_HEIGHT, BLACK);
        LCD_DrawString(10, 220, cpuUsageStr, &Font20, LGRAYBLUE, BLACK);

#ifdef DEBUG_LOGS_ENABLED
        LogPrintf(LOG_DEBUG, "CPU Usage: %lu %%", cpuUsage);

        statsBuffer[0] = '\r'; // Fix log formatting : [Timestamp][LOG_SEVERITY][task]
        statsBuffer[1] = '\n'; //                       RunTimeStats after new line

        vTaskGetRunTimeStats(&statsBuffer[2]);
        LogPrintf(LOG_DEBUG, "%s", statsBuffer);

        for (UBaseType_t i = 0; i < taskCount; i++)
        {
            LogPrintf(LOG_DEBUG, "Task [%s] high water mark: %lu", taskStatusArray[i].pcTaskName, taskStatusArray[i].usStackHighWaterMark);
        }

#else
        for (UBaseType_t i = 0; i < taskCount; i++)
        {
            if (taskStatusArray[i].usStackHighWaterMark <= HIGHWATER_MARK_MIN)
            {
                LogPrintf(LOG_WARNING "Task [%s] high water mark: %lu", taskStatusArray[i].pcTaskName, taskStatusArray[i].usStackHighWaterMark);
            }
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
