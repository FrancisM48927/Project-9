/**********************************************************************
* CPEG222 Project 9 - Servo PWM helper (header)
* Left servo PWM1 = PD12 (TIM4_CH1), Right servo PWM3 = PD14 (TIM4_CH3), AF2
* 20 ms period, pulse width 1280..1720 us (1500 us = stop)
**********************************************************************/
#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx.h"
#include <stdint.h>

#define SERVO_LEFT    0
#define SERVO_RIGHT   1

#define PULSE_STOP    1500U     // us: servo stopped
#define PULSE_SPAN    220U      // us: full speed is 220 us away from stop (~140 RPM)

void     Servo_Init(void);                          // starts both servos at 1500 us (stopped)
void     Servo_SetPulse(uint8_t servo, uint16_t us);
uint16_t Servo_GetPulse(uint8_t servo);             // the pulse being sent right now

#endif
