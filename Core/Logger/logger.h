/*
 * logger.h
 *
 *  Created on: Jan 25, 2026
 *      Author: Wojciech, Niewiadomski
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#pragma once

#include <stdarg.h>

#define LOGGER_BUFFER_SIZE 256
#define LOGGER_STREAM_SIZE 1024

typedef enum
{
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
} LogSeverity_t;

void LoggerInit(void);
void LogPrintf(LogSeverity_t severity, const char* fmt, ...);
void LoggerTask(void* argument);

#endif /* INC_LOGGER_H_ */
