/**********************************************************************
* CPEG222 Project 9 - Buttons helper (header)
* UP PF7, DOWN PE5, LEFT PF9, RIGHT PE6, CENTER PF8  (pressed = 0)
* Polled with a debounce + "lock" so one press gives exactly one event.
**********************************************************************/
#ifndef BUTTONS_H
#define BUTTONS_H

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum { BTN_NONE = 0, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_CENTER } Button;

void   Buttons_Init(void);
Button Buttons_Poll(uint32_t nowMs);   // returns a button ONCE per press, else BTN_NONE

#endif
