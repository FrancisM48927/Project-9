/**********************************************************************
* CPEG222 Project 9 - Servo feedback (TIM1 input capture) helper (header)
* Left feedback  = PE13 (TIM1_CH3, AF1)   Right feedback = PE9 (TIM1_CH1, AF1)
* Each feedback pin feeds TWO TIM1 channels:
*   right: CH1 = rising edge,  CH2 = falling edge (both from PE9)
*   left : CH3 = rising edge,  CH4 = falling edge (both from PE13)
* The TIM1_CC interrupt handler lives in main.c and calls Feedback_OnRise().
**********************************************************************/
#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "stm32f4xx.h"
#include <stdint.h>

void     Feedback_Init(void);
// Call from the TIM1 capture ISR at each RISING edge: 'rise' = this rising edge time,
// 'fall' = the falling edge time captured since the previous rising edge.
void     Feedback_OnRise(uint8_t servo, uint16_t rise, uint16_t fall);
uint16_t Feedback_Angle(uint8_t servo);   // latest angle in tenths of a degree (0..3600)
uint32_t Feedback_Count(uint8_t servo);   // number of good measurements so far

#endif
