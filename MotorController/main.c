/*
 * MotorController.c
 *
 * Created: 5/4/2019 5:21:59 PM
 * Author : Lee
 */ 

#define F_CPU 1000000
//#define CAPTURE_RPM
#define CAPTURE_PID

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>

typedef uint8_t bool;
const bool false = 0;
const bool true = 1;

#define ARRAY_COUNT(a) (sizeof(a) / sizeof(a[0]))

// Pin 13 - PA0 - INPUT - GOTO_MAXIMUM_PIN
//                Input indicating the desired position of the motor.
//                LOW - Drive motor to minimum
//                HIGH - Drive motor to maximum
//                PCINT0 enabled to wake the MCU on change.
// Pin 12 - PA1 - *** Available ***
// Pin 11 - PA2 - *** Available ***
// Pin 10 - PA3 - *** Available ***
// Pin  9 - PA4 - DEBUGGER ISP - USCK
// Pin  8 - PA5 - DEBUGGER ISP - MISO
// Pin  7 - PA6 - DEBUGGER ISP- MOSI
// Pin  6 - PA7 - OUTPUT - MOTOR CONTROL PWM
// Pin  2 - PB0 - OUTPUT - MOTOR CONTROL IN1
// Pin  3 - PB1 - OUTPUT - MOTOR CONTROL IN2
// Pin  5 - PB2 - INPUT - Hall effect sensor for motor tachometer
//                INT0 is enabled to generate an interrupt on the falling edge.
// Pin  4 - PB3 - DEBUGGER RESET - ISP and debugWire

// Forward declarations
int32_t GetTimerTick();
int32_t GetTimerTickChange(int32_t prevTick);
bool IsTimeout(int32_t tickTimeout);
bool ChangeMotorPower(int16_t powerChange);
bool SetMotorPower(int16_t power);
void SetMotorDirection(uint8_t direction);
int16_t GetMotorRPM();
bool MotorPIDControl();
void ResetMotorPIDControl();
void SetTargetRPM(int16_t targetRPM);

#define GOTO_MAXIMUM_PIN PA0
#define GOTO_MAXIMUM_MASK _BV(GOTO_MAXIMUM_PIN)

#define MOTOR_CONTROL_MASK (_BV(PB1) | _BV(PB0))
#define MOTOR_BRAKE 0
#define MOTOR_FORWARD	1
#define MOTOR_REVERSE	2
#define MOTOR_FLOAT	3
#define MOTOR_NO_DIRECTION 0xFF

#define HALL_EFFECT_PIN PB2
#define HALL_EFFECT_MASK _BV(HALL_EFFECT_PIN)

#define MOTOR_PWM_PIN PA7
#define MOTOR_PWM_MASK _BV(MOTOR_PWM_PIN)

#define CLOCK_SELECT_MASK		(_BV(CS02) | _BV(CS01) | _BV(CS00))
#define NO_CLOCK				0
#define CLOCK_NO_PRESCALE		1
#define CLOCK_PRESCALE_DIV_8	2
#define CLOCK_PRESCALE_DIV_64	3
#define CLOCK_PRESCALE_DIV_256	4
#define CLOCK_PRESCALE_DIV_1024 5
#define FALLING_EXTERNAL_CLOCK	6
#define RISING_EXTERNAL_CLOCK	7

#define TIMER1_PRESCALE CLOCK_PRESCALE_DIV_8

#if TIMER1_PRESCALE == CLOCK_NO_PRESCALE
#define TicksPerSecond (F_CPU)
#elif TIMER1_PRESCALE == CLOCK_PRESCALE_DIV_8
#define TicksPerSecond (F_CPU/8)
#elif TIMER1_PRESCALE == CLOCK_PRESCALE_DIV_64
#define TicksPerSecond (F_CPU/64)
#elif TIMER1_PRESCALE == CLOCK_PRESCALE_DIV_256
#define TicksPerSecond (F_CPU/256)
#elif TIMER1_PRESCALE == CLOCK_PRESCALE_DIV_1024
#define TicksPerSecond (F_CPU/1024)
#else
#error TIMER1_PRESCALE must be defined
#endif

// Extend TIMER1 to 32-bits by keeping track of 16-bit overflow
volatile uint16_t tim1High = 0;
ISR(TIM1_OVF_vect)
{
    tim1High++;
}

int16_t overflowCount = 0;
int32_t GetTimerTickChange(int32_t prevTick)
{
    return GetTimerTick() - prevTick;
}
bool IsTimeout(int32_t tickTimeout)
{
    return (GetTimerTick() >= tickTimeout) ? true : false;
}
int32_t GetTimerTick()
    {
    int32_t now = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
        if ((TIFR1 & _BV(TOV1)) != 0)
            {
            TIFR1 |= _BV(TOV1);  // Clear the overflow
            tim1High++;
            overflowCount++;
            }

        now = (((int32_t) tim1High) << 16) + TCNT1;
        }
        
    return now;
    }

// Implement a tachometer on the motor by keeping track of the number of timer1 ticks between pulses of the hall effect sensor.
// Keep 16 values for a rolling average.
volatile int32_t tachTickPrev = 0;
volatile uint8_t tachTicksIndex = 0;
#define tachSampleCount 32
volatile int32_t tachTicks[tachSampleCount] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile int16_t tachTicksOverflowCount = 0;
volatile int32_t tachTicksRollingTotal = 0;
volatile int32_t tachCount = 0;
volatile int32_t tachTicksRollingAverage = 0;

// ISR for Hall effect sensor on the motor.
ISR(EXT_INT0_vect)
    {
    int32_t tachNow = GetTimerTick();
    int32_t tachDelta = tachNow - tachTickPrev;
    if ((tachDelta & 0xFFFF0000) != 0)
        {
        tachTicksOverflowCount++;
        }
    tachCount++;

    // We keep an array of the last n tachTicks to compute a rolling average.
    tachTicksRollingTotal -= tachTicks[tachTicksIndex];  // about to overwrite this value... remove it from the rolling total
    tachTicks[tachTicksIndex] = tachDelta;
    tachTicksRollingTotal += tachDelta;
    int32_t rollingCount = (tachCount < tachSampleCount) ? tachCount : tachSampleCount;
    tachTicksRollingAverage = (tachTicksRollingTotal / rollingCount);
    
    tachTicksIndex++;
    // Wrap around at the end of the array
    if (tachTicksIndex >= ARRAY_COUNT(tachTicks))
        {
        tachTicksIndex = 0;
        }
    tachTickPrev = tachNow;
    }

const int32_t ticksPerMinute = 60*TicksPerSecond;

int16_t GetMotorRPM()
    {
    // Effectively zero
    if (tachTicksRollingAverage > 2*TicksPerSecond)
        {
        return 0;
        }

    // If there hasn't been a tachTick in a long while it's effectively zero.
    if ((tachCount == 0) || (GetTimerTickChange(tachTickPrev) > 2*TicksPerSecond))
        {
            return 0;
        }
    
    int32_t rpm = ticksPerMinute / tachTicksRollingAverage;
    
    return (int16_t) rpm;
    }

ISR(PCINT0_vect)
    {
    // GOTO_MAXIMUM_PIN changed state.
    // Do nothing... just wake up from sleep.
    }

const int16_t MotorMaxRPM = 7000;
const int16_t MotorMinPower = 64;
const int16_t MotorMaxPower = 255;

uint8_t motorDirection = MOTOR_NO_DIRECTION;
void SetMotorDirection(uint8_t direction)
{
    if (motorDirection != direction)
        {
        motorDirection = PORTB & ~MOTOR_CONTROL_MASK;
        motorDirection |= direction;
        PORTB = motorDirection;

        ResetMotorPIDControl();
        }
}

int16_t lastMotorPower;
bool ChangeMotorPower(int16_t powerChange)
{
    return SetMotorPower(lastMotorPower + powerChange);
}

bool SetMotorPower(int16_t power)
    {
    bool ok = true;
        
    if (power != lastMotorPower)
        {
        if (power > MotorMaxPower)
            {
            ok = false;   // Tried to over power the motor (probably because it stalled)
            power = MotorMaxPower;
            }
        else if (power < MotorMinPower)
            {
            power = MotorMinPower;
            }            
                        
        lastMotorPower = power;
            
        OCR0B = power;
        }
        
    return ok;
    }
int16_t finalTargetRPM = 0;       // The speed we ultimately want to get to.
int16_t targetRPM = 0;    // The speed we are currently targeting as we ramp the speed up/down.
uint8_t stepsUntilTargetRPM = 0;  // number of incremental changes left to get to the final target speed.

void SetTargetRPM(int16_t newFinalTargetRPM)
{
    finalTargetRPM = newFinalTargetRPM;
    stepsUntilTargetRPM = 1;
    targetRPM = (finalTargetRPM / stepsUntilTargetRPM);  // This is the first step.
    stepsUntilTargetRPM--;
}

int16_t MotorPowerForRPM(int16_t rpm)
{
    return (int16_t)(((int32_t) rpm)*MotorMaxPower/MotorMaxRPM);
}

int32_t timeoutTicks = 0;
bool IsActive()
{
    return !IsTimeout(timeoutTicks);
}
void Active(int32_t now)
{
    timeoutTicks = now + TicksPerSecond*10;
}

#if defined(CAPTURE_RPM)
    int16_t rpms[tachSampleCount];
#endif
int main(void)
    {
    DDRA &= ~(GOTO_MAXIMUM_MASK);  // Input
    DDRA |= MOTOR_PWM_MASK;	       // Output
    PORTA |= GOTO_MAXIMUM_MASK;    // Enable pull-up resistors on these input pin.
    
    DDRB &= ~(HALL_EFFECT_MASK);  // Input.
    DDRB |= MOTOR_CONTROL_MASK;   // Output
    PORTB |= HALL_EFFECT_MASK;    // Enable pull-up resistor.

    MCUCR &= ~(_BV(ISC01) | _BV(ISC00));
    MCUCR |= _BV(ISC01);   // ISC01 = 1, ISC00 = 0 - The falling edge of INT0 generates an interrupt request.
    GIMSK |= _BV(INT0);	   // Enable INT0.
    
    // Setup Timer 0 for PWM
    // WGM01 = 1, WGM00 = 1 - Fast PWM Wave Generation Mode
    // COM0B1 = 1, COM0B0 = 0 - OC0B high when count < OCR0B (higher OCR0B yields higher pulse width)
    TCCR0A &= ~(_BV(WGM01) | _BV(WGM00) | _BV(COM0B1) | _BV(COM0B0));  // Clear these bits in preparation for the following OR.
    TCCR0A |= _BV(WGM01) | _BV(WGM00) |  _BV(COM0B1);
    
    TCCR0B &= ~(CLOCK_SELECT_MASK);
    TCCR0B |= CLOCK_PRESCALE_DIV_64;

    // Run timer1 full blast.
    TCCR1A = 0;
    TCCR1B = TIMER1_PRESCALE;
    TIFR1 |= _BV(TOV1);     // Clear overflow
    TIMSK1 = _BV(TOIE1);    // Enable interrupts
    
    PCMSK0 |= _BV(PCINT0); // Enable port change interrupt on PA0 (GOTO_MAXIMUM_PIN)
    GIMSK |= PCIE0;

    sei();
    

    uint8_t prevTargetMaximum = 0xff;  // Start out with an invalid value
    
#if defined(CAPTURE_RPM)
    SetMotorDirection(MOTOR_REVERSE);
    SetMotorPower(255);

    int32_t stopTick = GetTimerTick() + 5*TicksPerSecond;
    
    while (!IsTimeout(stopTick))
        {
        }
    SetMotorPower(0);
    SetMotorDirection(MOTOR_FLOAT);
    for (int i = 0; i < tachSampleCount; i++)
        {
        int32_t rpm = ticksPerMinute / tachTicks[i];
        rpms[i] = (int16_t) rpm;
        }
#endif
    
    while (1) 
        {
        uint8_t pa = PINA;
        uint8_t targetMaximum = (pa & GOTO_MAXIMUM_MASK) == 0;
        uint8_t atMaximum = false;
        uint8_t atMinimum = false;

        if (targetMaximum != prevTargetMaximum)
            {
            prevTargetMaximum = targetMaximum;
            if (targetMaximum && !atMaximum)
                {
                // Want to drive the motor towards maximum limit.
                SetMotorDirection(MOTOR_FORWARD);
                SetTargetRPM(MotorMaxRPM);
                SetMotorPower(MotorPowerForRPM(MotorMaxRPM));
                atMinimum = false;  // We're driving the motor away from the minimum.
                }
            else if (!targetMaximum && !atMinimum)
                {
                // Want to drive the motor towards minimum limit.
                SetMotorDirection(MOTOR_REVERSE);
                SetTargetRPM(MotorMaxRPM);
                SetMotorPower(MotorPowerForRPM(MotorMaxRPM));
                atMaximum = false;  // We're driving the motor away from the maximum.
                }
            }
            
        if (!IsActive() && (GetMotorRPM() == 0))
            {
#if 0
            // Sleep the processor to save power.
            // It will wake up when the PCINT0 pin changes.
            cli();
            sleep_enable();
            sleep_bod_disable(); // disable brown out detect
            sei(); // next instruction (sleep_cpu()) is guaranteed to execute before interrupts are enabled.
            sleep_cpu();
            sleep_disable();
#endif
            }

        if (targetRPM > 0)
            {
            if (MotorPIDControl() == 0)
                {
                // Motor stalled... must have reached the end stop.
                atMaximum = targetMaximum;
                atMinimum = !targetMaximum;
                SetMotorPower(0);
                SetMotorDirection(MOTOR_BRAKE);
                SetTargetRPM(0);
                }
            }
        }
    }

// PID co-efficients expressed as fractions (e.g. Kp = Kpn/Kpd)    
int16_t Kpn = 0;    // Kp = 1/2
int16_t Kpd = 2;
int16_t Kin = 0;    // Ki = 1/4
int16_t Kid = 4;
int16_t Kdn = 0;    // Kd = 0/1
int16_t Kdd = 1;

int16_t integral = 0;
int16_t prevError = 0;
int32_t nextPIDTicks = 0;
int32_t nextRPMBoostTicks = 0;
uint8_t overpowerCount = 0;
int16_t maxRPM = 0;
uint8_t maxPower = 0;
int16_t fasterCount = 0;
int16_t slowerCount = 0;

#define CAPTURE_PID
#if defined(CAPTURE_PID)
#define PID_SAMPLES 64
uint8_t pidSampleIndex = 0;
int16_t rpmChanges[PID_SAMPLES] = {0};
int8_t powerChanges[PID_SAMPLES] = {0};
#endif
bool MotorPIDControl()
    {
    int32_t now = GetTimerTick();
    
    Active(now);
    
    if (now >= nextPIDTicks)
        {
        nextPIDTicks = now + TicksPerSecond/64;
        if (now >= nextRPMBoostTicks)
            {
            nextRPMBoostTicks = now + TicksPerSecond/32;
                
            if (stepsUntilTargetRPM > 0)
                {
                int16_t deltaRPM = (finalTargetRPM - targetRPM)/stepsUntilTargetRPM;
                stepsUntilTargetRPM--;
                targetRPM += deltaRPM;                
                }
            }            

        int16_t rpm = GetMotorRPM();
        int16_t error = targetRPM - rpm;
        
        if (rpm > targetRPM)
            {
            fasterCount++;
            }
            else
            {
            slowerCount++;
            }                    
        if (rpm > maxRPM)
            rpm = maxRPM;
            
        if ((maxRPM > finalTargetRPM/2) && (rpm < finalTargetRPM/2))
            return false;
        
        int16_t derivative = prevError - error;

        integral += error;
    
        int16_t rpmChange = Kpn*error/Kpd + Kin*integral/Kid + Kdn*derivative/Kdd;
        
        // Scale from RPM to power
        //int16_t powerChange = (int16_t)(((int32_t) rpmChange)*((int32_t)(MotorMaxPower - MotorMinPower))/(int32_t)(MotorMaxRPM - 0));
        int16_t powerChange = MotorPowerForRPM(rpmChange);

#if defined(CAPTURE_PID)
        rpmChanges[pidSampleIndex] = rpmChange;
        powerChanges[pidSampleIndex] = powerChange;
        pidSampleIndex++;
        if (pidSampleIndex >= ARRAY_COUNT(rpmChanges))
           {
           pidSampleIndex = 0;
           }
#endif        

        if (!ChangeMotorPower(powerChange))
            {
            overpowerCount++;
            
            return true; // (overpowerCount > 10) ? false : true;
            }

        overpowerCount = 0;
        }
        
    return true;
    }

void ResetMotorPIDControl()
    {
    nextPIDTicks = 0;
    nextRPMBoostTicks = 0;
    integral = 0;
    prevError = 0;
    maxRPM = 0;
    maxPower = 0;
    fasterCount = 0;
    slowerCount = 0;
    int32_t now = GetTimerTick();
    nextPIDTicks = now + TicksPerSecond/2;
    }
