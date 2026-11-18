/**********************************************************************
* CPEG222 Project 9 - Servo feedback (TIM1 input capture) helper (source)
*
* The servo sends back a 910 Hz PWM (cycle about 1.1 ms). Its DUTY CYCLE is
* the shaft angle:  2.9 % = 0 degrees ... 97.1 % = 360 degrees.
*
* TIM1 free-runs at 1 MHz (PSC = 179 on the 180 MHz timer clock, ARR = 0xFFFF),
* so one count = 1 us. Input capture saves the counter value at each edge:
*   rising edge  -> time of the start of a cycle
*   falling edge -> end of the high time
*   high  = fall - previous rise
*   cycle = rise - previous rise
* uint16_t subtraction handles the 16-bit counter wrapping around.
**********************************************************************/
#include "Feedback.h"

#define CYCLE_MIN_US   1000U     // ignore a cycle that is not close to 1.1 ms
#define CYCLE_MAX_US   1200U
#define DUTY_0_DEG     29U       // 2.9 %  in tenths of a percent
#define DUTY_360_DEG   971U      // 97.1 %

typedef struct {
    uint16_t prevRise;
    uint8_t  havePrev;
    volatile uint16_t angle;     // tenths of a degree
    volatile uint32_t count;     // good samples
} FbChannel;

static FbChannel fb[2];

void Feedback_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PE9 (TIM1_CH1) and PE13 (TIM1_CH3): alternate function, AF1
    GPIOE->MODER &= ~((3U << (9 * 2)) | (3U << (13 * 2)));
    GPIOE->MODER |=  ((2U << (9 * 2)) | (2U << (13 * 2)));
    GPIOE->AFR[1] &= ~((0xFU << ((9 - 8) * 4)) | (0xFU << ((13 - 8) * 4)));
    GPIOE->AFR[1] |=  ((1U   << ((9 - 8) * 4)) | (1U   << ((13 - 8) * 4)));

    // TIM1: free running 1 MHz counter
    TIM1->PSC = 180 - 1;
    TIM1->ARR = 0xFFFF;

    // CC1S = 01: CH1 captures TI1 (direct).   CC2S = 10: CH2 ALSO captures TI1 (indirect)
    // CC3S = 01: CH3 captures TI3 (direct).   CC4S = 10: CH4 ALSO captures TI3 (indirect)
    TIM1->CCMR1 = (1U << 0) | (2U << 8);
    TIM1->CCMR2 = (1U << 0) | (2U << 8);

    // CH1, CH3 capture the RISING edge (CCxP = 0). CH2, CH4 capture the FALLING edge (CCxP = 1)
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P |
                 TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC4P;

    // Interrupt on the rising-edge channels only (the falling time is already saved by then)
    TIM1->DIER = TIM_DIER_CC1IE | TIM_DIER_CC3IE;

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0;
    NVIC_SetPriority(TIM1_CC_IRQn, 0);       // timing-critical: highest priority
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    TIM1->CR1 |= TIM_CR1_CEN;
}

void Feedback_OnRise(uint8_t servo, uint16_t rise, uint16_t fall)
{
    FbChannel *f = &fb[servo];

    if (f->havePrev) {
        uint16_t cycle = (uint16_t)(rise - f->prevRise);   // rise - previous rise
        uint16_t high  = (uint16_t)(fall - f->prevRise);   // fall - previous rise

        if (cycle >= CYCLE_MIN_US && cycle <= CYCLE_MAX_US && high < cycle) {
            uint32_t duty = ((uint32_t)high * 1000U) / cycle;       // tenths of a percent
            if (duty < DUTY_0_DEG)   duty = DUTY_0_DEG;
            if (duty > DUTY_360_DEG) duty = DUTY_360_DEG;
            f->angle = (uint16_t)(((duty - DUTY_0_DEG) * 3600U) / (DUTY_360_DEG - DUTY_0_DEG)); // /942
            f->count++;
        }
    }
    f->prevRise = rise;
    f->havePrev = 1;
}

uint16_t Feedback_Angle(uint8_t servo) { return fb[servo].angle; }
uint32_t Feedback_Count(uint8_t servo) { return fb[servo].count; }
