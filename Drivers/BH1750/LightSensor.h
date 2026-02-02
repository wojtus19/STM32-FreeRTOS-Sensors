/*
 * LightSensor.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Wojtus
 */

#pragma once
#ifndef BH1750_LIGHTSENSOR_H_
#define BH1750_LIGHTSENSOR_H_

#include "typedef.h"
#include <stdint.h>

#define BH1750_ADDRESS (0x23 << 1)

Status_t BH1750_Init();
Status_t BH1750_ReadLux(float* lux);
Status_t BH1750_Deinit();

#endif /* BH1750_LIGHTSENSOR_H_ */
