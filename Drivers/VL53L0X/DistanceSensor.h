
#include "stm32f4xx_hal.h"
#include "typedef.h"

#define VL530L0X_ADDRESS (0x29 << 1)
#define VL53_XSHUT_GPIO_Port GPIOC
#define VL53_XSHUT_Pin GPIO_PIN_1

typedef enum vcselPeriodType_t
{
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vcselPeriodType_t;

Status_t Init_VL53L0X(uint8_t b_long_range);
uint16_t readRangeSingleMillimeters(void);
uint8_t setVcselPulsePeriod(vcselPeriodType_t type, uint8_t period_pclks);
float getSignalRateLimit();
void startContinuous(uint32_t period_ms);
