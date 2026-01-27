#pragma once
#include <stdint.h>

static inline int IS_RAM_ADDRESS(const void *ptr)
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
