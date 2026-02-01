#pragma once

#include "stdint.h"

#define FALSE 0
#define TRUE 1

typedef uint8_t bool_t;

typedef enum Status_t
{
    STATUS_OK,
    STATUS_INVALID_PARAMS,
    STATUS_TIMEOUT_ERROR,
    STATUS_I2C_ERROR,
    STATUS_ERROR
} Status_t;
