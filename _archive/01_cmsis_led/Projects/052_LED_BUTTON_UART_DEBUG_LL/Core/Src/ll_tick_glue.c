// Core/Src/ll_tick_glue.c
#include "stm32f1xx.h"

__IO uint32_t uwTick;

void LL_IncTick(void) {
    uwTick++;
}

uint32_t LL_GetTick(void) {
    return uwTick;
}
