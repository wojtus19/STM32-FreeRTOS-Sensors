#include "utils.h"
#include <stdio.h>

bool_t IsRamAddress(const void* ptr)
{
    uint32_t addr = (uint32_t)ptr;

    /* SRAM1 + SRAM2 */
    if ((addr >= 0x20000000UL) && (addr <= 0x2001FFFFUL))
        return 1;

    /* Flash (nie DMA) */
    if ((addr >= 0x08000000UL) && (addr <= 0x080FFFFFUL))
        return 0;

    /* CCM RAM (nie DMA!) */
    if ((addr >= 0x10000000UL) && (addr <= 0x1000FFFFUL))
        return 0;

    return 0;
}

void AccumToString(_Accum value, char* buffer, int precision)
{
    int is_negative = 0;
    if (value < 0)
    {
        is_negative = 1;
        value       = -value;
    }
    int32_t int_part = (int32_t)value;

    _Accum frac_part = value - int_part;

    uint32_t frac_scaled = 0;
    _Accum scale         = 1;
    for (uint8_t i = 0; i < precision; i++)
        scale *= 10;
    frac_scaled = (uint32_t)(frac_part * scale + 0.5k);

    if (is_negative)
        sprintf(buffer, "-%" PRId32 ".%0*" PRIu32, int_part, precision, frac_scaled);
    else
        sprintf(buffer, "%" PRId32 ".%0*" PRIu32, int_part, precision, frac_scaled);
}

void ReverseBuffer(uint8_t* buffer, uint8_t len)
{
    uint8_t tempVal;

    for (uint8_t i = 0, j = (len - 1); i < j; i++, j--)
    {
        tempVal   = buffer[i];
        buffer[i] = buffer[j];
        buffer[j] = tempVal;
    }
}