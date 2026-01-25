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

void LoggerInit(void);
void LogPrintf(const char* fmt, ...);
void LoggerTask(void* argument);

#endif /* INC_LOGGER_H_ */
