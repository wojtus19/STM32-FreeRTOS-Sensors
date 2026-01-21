#pragma once

#define VL530L0X_ADDRESS 0x52u

#define FALSE 0
#define TRUE 1

typedef uint8_t bool_t;

typedef enum Status_t
{
	OK,
	ACK_OK,
	TIMEOUT_ERROR,
	NO_ACK_I2C,
	DEV_NOT_READY,
	OUT_OF_RANGE,
	NOT_IMPLEMENTED
}Status_t;
