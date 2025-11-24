#ifndef LL_TICK_H
#define LL_TICK_H
#include "stm32f1xx.h"
#ifdef __cplusplus
extern "C" {
#endif
uint32_t LL_GetTick(void);
void     LL_IncTick(void);
#ifdef __cplusplus
}
#endif
#endif
