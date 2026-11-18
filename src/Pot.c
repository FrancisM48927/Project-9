/**********************************************************************
* CPEG222 Project 9 - Potentiometer helper (source)
* PC2 is analog input ADC3 channel 12. The ADC turns 0..3.3 V into 0..4095.
* Software-triggered single conversions, averaged to reduce noise.
**********************************************************************/
#include "Pot.h"

#ifndef ADC123_COMMON
#define ADC123_COMMON ((ADC_Common_TypeDef *)(APB2PERIPH_BASE + 0x2300UL))
#endif

#define POT_CHANNEL  12U

void Pot_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

    GPIOC->MODER |= (3U << (2 * 2));                   // PC2 analog mode (11)

    // ADC clock = APB2 (90 MHz) / 4 = 22.5 MHz (must stay below 36 MHz)
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC3->CR1 = 0;
    ADC3->CR2 = 0;
    ADC3->SMPR1 = (ADC3->SMPR1 & ~(7U << ((POT_CHANNEL - 10) * 3)))
                | (7U << ((POT_CHANNEL - 10) * 3));    // longest sample time (480 cycles)
    ADC3->SQR1 = 0;                                    // 1 conversion in the sequence
    ADC3->SQR3 = POT_CHANNEL;                          // ...and it is channel 12
    ADC3->CR2 |= ADC_CR2_ADON;                         // turn the ADC on
    for (volatile uint32_t i = 0; i < 1000; i++) { }   // let it power up
}

uint16_t Pot_Read(void)
{
    uint32_t sum = 0;
    for (uint8_t n = 0; n < 8; n++) {
        ADC3->CR2 |= ADC_CR2_SWSTART;                  // start one conversion
        while (!(ADC3->SR & ADC_SR_EOC)) { }           // wait until finished
        sum += ADC3->DR;                               // reading DR clears EOC
    }
    return (uint16_t)(sum / 8U);
}
