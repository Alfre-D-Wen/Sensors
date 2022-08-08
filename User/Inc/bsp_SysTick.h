#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f1xx.h"

void SysTick_Init(void);
void Delay_us(__IO uint32_t nTime); // 单位1us

#define Delay_ms(x) Delay_us(1000 * x) //单位ms

#endif /* __SYSTICK_H */
