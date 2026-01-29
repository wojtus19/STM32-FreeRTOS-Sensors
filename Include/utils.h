#pragma once
#include "typedef.h"
#include <inttypes.h>
#include <stdint.h>

bool_t IS_RAM_ADDRESS(const void* ptr);
void AccumToString(_Accum value, char* buffer, int precision);
void ReverseBuffer(uint8_t* buffer, uint8_t len);