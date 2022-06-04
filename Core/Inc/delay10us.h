#ifndef MICROTIMER_H_INCLUDED
#define MICROTIMER_H_INCLUDED

#include "stdbool.h"
#include "main.h"

bool Delay_100us(uint16_t Time);
void Delay_100usInit(void);
uint32_t TimerSet(const uint32_t AddTimeMs);
uint32_t TimerRemainingMs(const uint32_t Timer);
uint32_t TimerPassMs(const uint32_t Timer);

volatile static uint32_t TimeMs;

#endif /* MICROTIMER_H_INCLUDED */
