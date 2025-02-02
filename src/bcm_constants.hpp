#ifndef __BCM_CONSTANTS_H__
#define __BCM_CONSTANTS_H__

#include <stdint.h>

#include "bcm_config_constants.hpp"

// Refresh rate = F_CPU / prescaler / 256 (8-bit)
// Interrupt timer resolution = 1 / (F_CPU / prescaler)

#if MICROS_TIMER == MICROS_TIMER0

// Timer0
#if NUMBER_OF_BITS <= 8
#if PRESCALER == 256
#define CLOCKSEL (_BV(CS02))

#elif PRESCALER == 64
#define CLOCKSEL (_BV(CS01) | _BV(CS00))

#elif PRESCALER == 8
#define CLOCKSEL (_BV(CS01))

#elif PRESCALER == 1
#define CLOCKSEL (_BV(CS00))

#else
#error "This Timer doesn't have such prescaler."
#endif

#define REG_TCCRA TCCR0A
#define REG_TCCRB TCCR0B
#define REG_TIMSK TIMSK0
#define REG_TCNT TCNT0
#define REG_OCR OCR0A
#define BIT_WGM WGM01
#define BIT_OCIE OCIE0A
#define ISR_VECT TIMER0_COMPA_vect
#define pwr_enable() power_timer0_enable()
#define pwr_disable() power_timer0_disable()

#define SET_TCCRA() (REG_TCCRA = _BV(BIT_WGM))
#define SET_TCCRB() (REG_TCCRB = CLOCKSEL)
#else
#error "Timer 0 has only 8 bits. Select Timer 1 instead."
#endif

#elif MICROS_TIMER == MICROS_TIMER1

// Timer1

#if PRESCALER == 1024
#define CLOCKSEL (_BV(CS12) | _BV(CS10))

#elif PRESCALER == 256
#define CLOCKSEL (_BV(CS12))

#elif PRESCALER == 64
#define CLOCKSEL (_BV(CS11) | _BV(CS10))

#elif PRESCALER == 8
#define CLOCKSEL (_BV(CS11))

#elif PRESCALER == 1
#define CLOCKSEL (_BV(CS10))

#else
#error "This Timer doesn't have such prescaler."
#endif

#define REG_TCCRA TCCR1A
#define REG_TCCRB TCCR1B
#define REG_TIMSK TIMSK1
#define REG_TCNT TCNT1
#define REG_OCR OCR1A
#define BIT_WGM WGM12
#define BIT_OCIE OCIE1A
#define ISR_VECT TIMER1_COMPA_vect
#define pwr_enable() power_timer1_enable()
#define pwr_disable() power_timer1_disable()

#define SET_TCCRA() (REG_TCCRA = 0)
#define SET_TCCRB() (REG_TCCRB = _BV(BIT_WGM) | CLOCKSEL)

#elif MICROS_TIMER == MICROS_TIMER2

// Timer2
#if NUMBER_OF_BITS <= 8
#if PRESCALER == 256
#define CLOCKSEL (_BV(CS22) | _BV(CS21))

#elif PRESCALER == 128
#define CLOCKSEL (_BV(CS22) | _BV(CS20))

#elif PRESCALER == 64
#define CLOCKSEL (_BV(CS22))

#elif PRESCALER == 32
#define CLOCKSEL (_BV(CS21) | _BV(CS20))

#elif PRESCALER == 8
#define CLOCKSEL (_BV(CS21))

#elif PRESCALER == 1
#define CLOCKSEL (_BV(CS20))

#else
#error "This Timer doesn't have such prescaler."
#endif

#define REG_TCCRA TCCR2A
#define REG_TCCRB TCCR2B
#define REG_TIMSK TIMSK2
#define REG_TCNT TCNT2
#define REG_OCR OCR2A
#define BIT_WGM WGM21
#define BIT_OCIE OCIE2A
#define ISR_VECT TIMER2_COMPA_vect
#define pwr_enable() power_timer2_enable()
#define pwr_disable() power_timer2_disable()

#define SET_TCCRA() (REG_TCCRA = _BV(BIT_WGM))
#define SET_TCCRB() (REG_TCCRB = CLOCKSEL)
#else
#error "Timer 2 has only 8 bits. Select Timer 1 instead."
#endif

#else
#error "Bad MICROS_TIMER set"
#endif
#endif // __BCM_CONSTANTS_H__