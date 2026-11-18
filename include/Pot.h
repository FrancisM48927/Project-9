/**********************************************************************
* CPEG222 Project 9 - Potentiometer helper (header)
* V_POT = PC2 = ADC3_IN12, 12-bit result 0..4095
**********************************************************************/
#ifndef POT_H
#define POT_H

#include "stm32f4xx.h"
#include <stdint.h>

void     Pot_Init(void);
uint16_t Pot_Read(void);     // average of 8 conversions, 0..4095

#endif
