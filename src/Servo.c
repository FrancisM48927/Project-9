/**********************************************************************
* CPEG222 Project 9 - Servo PWM helper (source)
*
* A continuous-rotation servo is told its speed by the WIDTH of a pulse that
* repeats every 20 ms (50 Hz):  1500 us = stop, more or less = turn faster
* one way or the other.
*
* TIM4 is on APB1 (45 MHz bus, timer clock doubled = 90 MHz).
*   PSC = 89  -> 90 MHz / 90 = 1 MHz = 1 count per microsecond
*   ARR = 19999 -> 20,000 counts = 20 ms period
*   CCRx = pulse width in microseconds (the PWM "high" time)
**********************************************************************/
#include "Servo.h"

#define PULSE_MIN  (PULSE_STOP - PULSE_SPAN)   // 1280
#define PULSE_MAX  (PULSE_STOP + PULSE_SPAN)   // 1720

static uint16_t pulse[2] = { PULSE_STOP, PULSE_STOP };

void Servo_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // PD12, PD14 -> alternate function (10), AF2 = TIM4
    GPIOD->MODER   &= ~((3U << (12 * 2)) | (3U << (14 * 2)));
    GPIOD->MODER   |=  ((2U << (12 * 2)) | (2U << (14 * 2)));
    GPIOD->AFR[1]  &= ~((0xFU << ((12 - 8) * 4)) | (0xFU << ((14 - 8) * 4)));
    GPIOD->AFR[1]  |=  ((2U   << ((12 - 8) * 4)) | (2U   << ((14 - 8) * 4)));

    TIM4->PSC = 90 - 1;                    // 1 MHz timer tick (1 us)
    TIM4->ARR = 20000 - 1;                 // 20 ms period
    TIM4->CCR1 = PULSE_STOP;               // start stopped
    TIM4->CCR3 = PULSE_STOP;

    // CH1 and CH3: PWM mode 1 (OCxM = 110) with preload
    TIM4->CCMR1 &= ~0xFFU;
    TIM4->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE;
    TIM4->CCMR2 &= ~0xFFU;
    TIM4->CCMR2 |= (6U << 4) | TIM_CCMR2_OC3PE;
    TIM4->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM4->CR1   |= TIM_CR1_ARPE;
    TIM4->EGR    = TIM_EGR_UG;             // load the registers
    TIM4->CR1   |= TIM_CR1_CEN;
}

void Servo_SetPulse(uint8_t servo, uint16_t us)
{
    if (us < PULSE_MIN) us = PULSE_MIN;
    if (us > PULSE_MAX) us = PULSE_MAX;
    pulse[servo] = us;
    if (servo == SERVO_LEFT) TIM4->CCR1 = us;
    else                     TIM4->CCR3 = us;
}

uint16_t Servo_GetPulse(uint8_t servo)
{
    return pulse[servo];
}
