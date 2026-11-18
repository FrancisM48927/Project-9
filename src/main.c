/**********************************************************************
Author: F. MAILOM
CPEG222 Project 9, 11/13/26
NucleoF466ZE CMSIS Control 360 Servo speed and direction
**********************************************************************/
#include "stm32f4xx.h"
#include "SysClock180MHz.h"
#include "ssd.h"
#include "OLED.h"
#include "Servo.h"
#include "Feedback.h"
#include "Pot.h"
#include "Buttons.h"

#define CONTROL_PERIOD_MS  100U     // control loop + OLED update every 100 ms
#define MAX_RPM            140U
#define MIN_RPM            5U       // a target below this is treated as STOP

// ------------------------- Shared with interrupt handlers -------------------------
static volatile uint32_t sysMs       = 0;   // free-running millisecond clock
static volatile uint32_t msCount     = 0;
static volatile uint8_t  controlFlag = 0;   // set every 100 ms, cleared by main()

// SysTick fires every 1 ms. One SysTick can only count ~93 ms at 180 MHz,
// so we count 100 ticks in software to get 100 ms.
void SysTick_Handler(void)
{
    sysMs++;
    if (++msCount >= CONTROL_PERIOD_MS) {
        msCount     = 0;
        controlFlag = 1;
    }
}

// TIM5: refresh one SSD digit per interrupt
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

// TIM1 capture: fires at the RISING edge of each feedback signal.
// CCR1/CCR3 = time of this rising edge, CCR2/CCR4 = time of the falling edge before it.
// (Reading CCRx also clears its interrupt flag.)
void TIM1_CC_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;
    if (sr & TIM_SR_CC1IF) {                              // right servo (PE9)
        uint16_t rise = (uint16_t)TIM1->CCR1;
        uint16_t fall = (uint16_t)TIM1->CCR2;
        Feedback_OnRise(SERVO_RIGHT, rise, fall);
    }
    if (sr & TIM_SR_CC3IF) {                              // left servo (PE13)
        uint16_t rise = (uint16_t)TIM1->CCR3;
        uint16_t fall = (uint16_t)TIM1->CCR4;
        Feedback_OnRise(SERVO_LEFT, rise, fall);
    }
}

// ------------------------- Mode logic -------------------------
typedef enum {
    MODE_STOP, MODE_FORWARD, MODE_REVERSE,
    MODE_LEFT_TURN, MODE_RIGHT_TURN, MODE_TANK_LEFT, MODE_TANK_RIGHT
} Mode;

static const char *modeName[] = {
    "STOP", "FORWARD", "REVERSE", "LEFT TURN", "RIGHT TURN", "TANK LEFT", "TANK RIGHT"
};

static Mode    mode = MODE_STOP;
static int8_t  dir[2]  = { 0, 0 };       // per servo: +1 forward, -1 reverse, 0 stopped
static int8_t  prevDir[2] = { 0, 0 };

static void SetMode(Mode m)
{
    mode = m;
    switch (m) {
    case MODE_STOP:        dir[SERVO_LEFT] =  0; dir[SERVO_RIGHT] =  0; break;
    case MODE_FORWARD:     dir[SERVO_LEFT] =  1; dir[SERVO_RIGHT] =  1; break;
    case MODE_REVERSE:     dir[SERVO_LEFT] = -1; dir[SERVO_RIGHT] = -1; break;
    case MODE_LEFT_TURN:   dir[SERVO_LEFT] =  0; dir[SERVO_RIGHT] =  1; break;
    case MODE_RIGHT_TURN:  dir[SERVO_LEFT] =  1; dir[SERVO_RIGHT] =  0; break;
    case MODE_TANK_LEFT:   dir[SERVO_LEFT] = -1; dir[SERVO_RIGHT] =  1; break;  // extra credit
    case MODE_TANK_RIGHT:  dir[SERVO_LEFT] =  1; dir[SERVO_RIGHT] = -1; break;  // extra credit
    }
}

// New button press. lastBtn lets us spot the 2nd consecutive LEFT/RIGHT (tank turn).
static void HandleButton(Button b)
{
    static Button lastBtn = BTN_NONE;

    switch (b) {
    case BTN_CENTER: SetMode(MODE_STOP);    break;
    case BTN_UP:     SetMode(MODE_FORWARD); break;
    case BTN_DOWN:   SetMode(MODE_REVERSE); break;
    case BTN_LEFT:
        if (lastBtn == BTN_LEFT && mode == MODE_LEFT_TURN)      SetMode(MODE_TANK_LEFT);
        else if (lastBtn == BTN_LEFT && mode == MODE_TANK_LEFT) { /* 3rd press ignored */ }
        else                                                    SetMode(MODE_LEFT_TURN);
        break;
    case BTN_RIGHT:
        if (lastBtn == BTN_RIGHT && mode == MODE_RIGHT_TURN)      SetMode(MODE_TANK_RIGHT);
        else if (lastBtn == BTN_RIGHT && mode == MODE_TANK_RIGHT) { /* 3rd press ignored */ }
        else                                                      SetMode(MODE_RIGHT_TURN);
        break;
    default: break;
    }
    lastBtn = b;                       // any other button breaks the LEFT/RIGHT streak
}

// ------------------------- Closed-loop speed control -------------------------
static uint16_t target = 0;            // target RPM from the pot (0 = stop)
static uint16_t rpm[2] = { 0, 0 };     // measured RPM (magnitude) of each servo
static int16_t  corr[2] = { 0, 0 };    // correction: how far the pulse is from 1500 us (0..220)
static uint8_t  moving[2] = { 0, 0 };
static uint8_t  settle[2] = { 0, 0 };  // ticks to wait after a start/reversal before correcting

// Update one servo's pulse. 'fresh' = we have a new RPM measurement to act on.
static void UpdateServo(uint8_t i, uint8_t fresh)
{
    if (dir[i] != prevDir[i]) { settle[i] = 2; }        // direction changed: let it spin up
    prevDir[i] = dir[i];

    if (dir[i] == 0 || target == 0) {                   // stopped (button, or tiny target)
        corr[i] = 0;
        moving[i] = 0;
    } else if (!moving[i]) {                            // just starting: first guess
        moving[i] = 1;
        settle[i] = 2;
        corr[i] = (int16_t)((target * PULSE_SPAN) / MAX_RPM);   // ~1.57 us per RPM
    } else if (settle[i] > 0) {
        if (fresh) settle[i]--;
    } else if (fresh) {
        int16_t err = (int16_t)target - (int16_t)rpm[i];        // + = too slow, - = too fast
        if (err > 1 || err < -1) {                              // +/-1 RPM dead band
            int16_t step = (int16_t)((err * 3) / 4);            // proportional step
            if (step == 0) step = (err > 0) ? 1 : -1;
            corr[i] += step;                                    // slow: further from 1500, fast: closer
            if (corr[i] < 0)                    corr[i] = 0;
            if (corr[i] > (int16_t)PULSE_SPAN)  corr[i] = (int16_t)PULSE_SPAN;
        }
    }

    // The servos are mounted facing opposite ways:
    //   left servo forward  = pulse ABOVE 1500 us,  right servo forward = pulse BELOW 1500 us
    int16_t offset = (int16_t)(dir[i] * corr[i]);
    int16_t pulse  = (i == SERVO_LEFT) ? (int16_t)(PULSE_STOP + offset)
                                       : (int16_t)(PULSE_STOP - offset);
    Servo_SetPulse(i, (uint16_t)pulse);
}

// ------------------------- OLED -------------------------
// Write 'v' right-aligned in 'width' characters; returns the new end pointer
static char *PutNum(char *p, uint32_t v, int width)
{
    char tmp[8];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10U); v /= 10U; } while (v && n < 7);
    for (int pad = width - n; pad > 0; pad--) *p++ = ' ';
    while (n) *p++ = tmp[--n];
    return p;
}

static void PadAndPrint(uint8_t page, char *buf, char *end)
{
    while (end < buf + 20) *end++ = ' ';
    *end = '\0';
    OLED_Print(page, 0, buf);
}

static void ShowMode(void)
{
    char buf[24];
    char *p = buf;
    const char *s = modeName[mode];
    while (*s) *p++ = *s++;
    PadAndPrint(0, buf, p);
}

static void ShowServo(uint8_t page, char side, uint8_t servo)
{
    char buf[24];
    char *p = buf;
    const char *a = " PWM ";
    const char *b = " RPM ";
    *p++ = side;
    while (*a) *p++ = *a++;
    p = PutNum(p, Servo_GetPulse(servo), 4);       // actual pulse being sent (us)
    while (*b) *p++ = *b++;
    p = PutNum(p, rpm[servo], 3);                  // actual measured RPM
    PadAndPrint(page, buf, p);
}

// ------------------------- Every 100 ms -------------------------
static void ControlTick(void)
{
    static uint8_t  primed = 0;
    static uint16_t lastAngle[2];
    static uint32_t lastCount[2];

    // ---- 1. Pot -> target RPM (0..140). Below 5 RPM counts as stop.
    uint32_t t = ((uint32_t)Pot_Read() * MAX_RPM) / 4095U;
    target = (t < MIN_RPM) ? 0U : (uint16_t)t;
    SSD_DisplayInt(target);

    // ---- 2. Angle readings -> RPM
    uint8_t fresh[2] = { 0, 0 };
    for (uint8_t i = 0; i < 2; i++) {
        uint16_t a = Feedback_Angle(i);
        uint32_t c = Feedback_Count(i);
        if (primed && c != lastCount[i]) {                 // new feedback arrived
            int32_t d = (int32_t)a - (int32_t)lastAngle[i];   // tenths of a degree in 100 ms
            if (d >  1800) d -= 3600;                      // crossed 360 -> 0 (wrap)
            if (d < -1800) d += 3600;
            if (d < 0) d = -d;                             // speed only; direction is known
            rpm[i] = (uint16_t)((d + 3) / 6);              // RPM = tenths-of-degree per 100 ms / 6
            fresh[i] = 1;
        } else if (primed) {
            rpm[i] = 0;                                    // no feedback (servo power off?)
        }
        lastAngle[i] = a;
        lastCount[i] = c;
    }
    primed = 1;

    // ---- 3. Control loop and display
    UpdateServo(SERVO_LEFT,  fresh[SERVO_LEFT]);
    UpdateServo(SERVO_RIGHT, fresh[SERVO_RIGHT]);

    ShowMode();
    ShowServo(2, 'L', SERVO_LEFT);
    ShowServo(4, 'R', SERVO_RIGHT);
}

// ------------------------- main -------------------------
int main(void)
{
    SystemClock_Config();                        // 16 MHz -> 180 MHz
    SystemCoreClockUpdate();
    Servo_Init();                                // both servos start at 1500 us = stopped
    SysTick_Config(SystemCoreClock / 1000);      // 1 ms tick (180000 counts)

    SSD_Init();
    OLED_Init();
    Feedback_Init();
    Pot_Init();
    Buttons_Init();

    SetMode(MODE_STOP);                          // robot starts stopped at reset
    SSD_DisplayInt(0);
    ShowMode();
    OLED_Print(2, 0, "L PWM 1500 RPM   0 ");
    OLED_Print(4, 0, "R PWM 1500 RPM   0 ");

    while (1)
    {
        // Buttons: a new press changes direction right away
        Button b = Buttons_Poll(sysMs);
        if (b != BTN_NONE) {
            HandleButton(b);
            UpdateServo(SERVO_LEFT, 0);
            UpdateServo(SERVO_RIGHT, 0);
            ShowMode();
        }

        // Every 100 ms (flag set by SysTick): measure speed, run control loop, update displays
        if (controlFlag) {
            controlFlag = 0;
            ControlTick();
        }
    }
}
