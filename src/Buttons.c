/**********************************************************************
* CPEG222 Project 9 - Buttons helper (source)
*
* Lock technique, as a small state machine (called every main-loop pass):
*   IDLE     : nothing pressed. A press starts the debounce timer.
*   DEBOUNCE : after 20 ms the same button must still be down -> report it ONCE.
*   HELD     : locked. Nothing new is reported until the button is released.
*   RELEASE  : button must stay released for 20 ms before the lock opens.
**********************************************************************/
#include "Buttons.h"

#define DEBOUNCE_MS 20U

typedef enum { ST_IDLE, ST_DEBOUNCE, ST_HELD, ST_RELEASE } BtnState;

void Buttons_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN;

    // Inputs (00) with pull-ups (01): PF7, PF8, PF9, PE5, PE6
    GPIOF->MODER &= ~((3U << (7 * 2)) | (3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOE->MODER &= ~((3U << (5 * 2)) | (3U << (6 * 2)));
    GPIOF->PUPDR &= ~((3U << (7 * 2)) | (3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOF->PUPDR |=  ((1U << (7 * 2)) | (1U << (8 * 2)) | (1U << (9 * 2)));
    GPIOE->PUPDR &= ~((3U << (5 * 2)) | (3U << (6 * 2)));
    GPIOE->PUPDR |=  ((1U << (5 * 2)) | (1U << (6 * 2)));
}

// Which button is down right now? (pressed = 0)
static Button Raw(void)
{
    if (!(GPIOF->IDR & (1U << 8))) return BTN_CENTER;
    if (!(GPIOF->IDR & (1U << 7))) return BTN_UP;
    if (!(GPIOE->IDR & (1U << 5))) return BTN_DOWN;
    if (!(GPIOF->IDR & (1U << 9))) return BTN_LEFT;
    if (!(GPIOE->IDR & (1U << 6))) return BTN_RIGHT;
    return BTN_NONE;
}

Button Buttons_Poll(uint32_t nowMs)
{
    static BtnState state = ST_IDLE;
    static Button   candidate = BTN_NONE;
    static uint32_t t0 = 0;
    Button raw = Raw();

    switch (state) {
    case ST_IDLE:
        if (raw != BTN_NONE) { candidate = raw; t0 = nowMs; state = ST_DEBOUNCE; }
        break;
    case ST_DEBOUNCE:
        if ((nowMs - t0) >= DEBOUNCE_MS) {
            if (raw == candidate) { state = ST_HELD; return candidate; }   // real press
            state = ST_IDLE;                                               // was a bounce
        }
        break;
    case ST_HELD:
        if (raw == BTN_NONE) { t0 = nowMs; state = ST_RELEASE; }
        break;
    case ST_RELEASE:
        if (raw != BTN_NONE)                 state = ST_HELD;    // bounced while releasing
        else if ((nowMs - t0) >= DEBOUNCE_MS) state = ST_IDLE;   // lock opens
        break;
    }
    return BTN_NONE;
}
