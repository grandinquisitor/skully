#include "bcm.hpp"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>

#include "constants.h"
#include "bcm_config_constants.hpp"
#include "bcm_constants.hpp"
#include "bcm_types.hpp"

volatile uint8_t BCM_CYCLE_END = 0;

static volatile leds_t
    timesliceB[NUMBER_OF_BITS];  // one byte for each bit-position being
                                 // displayed on a port
static volatile leds_t
    timesliceC[NUMBER_OF_BITS];  // one byte for each bit-position being
                                 // displayed on a port
static volatile leds_t
    timesliceD[NUMBER_OF_BITS];  // one byte for each bit-position being
                                 // displayed on a port

/*************************************************************
        FUNCTIONS
**************************************************************/
void BCM_init(void) {
  // Timer settings
  SET_TCCRA();
  SET_TCCRB();
  REG_TIMSK = _BV(BIT_OCIE);
  REG_OCR = 1;
  sei();
}

void BCM_stop(void) {
  REG_TIMSK &= ~_BV(BIT_OCIE);
  REG_TCCRB = 0;
}

void BCM_encode(duty_t dutyCycle[], char portLetter, const uint8_t LEDPins[],
                uint8_t nrOfLEDs) {
  leds_t bitmask;
  uint8_t bitposition;

  for (bitposition = 0, bitmask = 1; bitposition < NUMBER_OF_BITS;
       bitposition++, bitmask <<= 1) {
    leds_t portbits = 0;

    for (uint8_t ledpos = 0; ledpos < nrOfLEDs; ledpos++) {
      if (((dutyCycle[ledpos] & bitmask) != 0) ^ INVERT_POLARITY) {
        portbits |= 1 << LEDPins[ledpos];
      }
    }

    switch (portLetter) {
      case 'B':
        timesliceB[bitposition] =
            portbits;  // which leds are to be on at this bit position
        break;
      case 'C':
        timesliceC[bitposition] = portbits;
        break;
      case 'D':
        timesliceD[bitposition] = portbits;
        break;
    }
  }
}

/*************************************************************
        ISR Handlers
**************************************************************/

ISR(ISR_VECT) {
  static uint8_t bitpos = 0;  // which bit position is currently being shown
  bitpos++;

  if (bitpos > NUMBER_OF_BITS - 1) bitpos = 0;
  // bitpos &= 7; // reset to 0 if > 7 (doesn't wotk for all numbers like 9)

  // Re-trigger after 1, 2, 4, 8, 16... cycles
  REG_OCR <<= 1;

  if (bitpos == 0) {
    REG_OCR = 1;
  } else if (bitpos == NUMBER_OF_BITS - 1) {
    BCM_CYCLE_END = 1;  // flag for main loop
  }

  // Turn off then on only the led pins
  if (LED_PORTB_PIN_BM != 0) {
    if (LED_PORTB_PIN_BM == UINT8_MAX) {
      // if every LED is used, don't need to mask out unused pins
      // (the compiler might be smart enough to figure this out anyway)
      PORTB = timesliceB[bitpos];
    } else {
      PORTB = (PORTB & (~(LED_PORTB_PIN_BM))) | timesliceB[bitpos];
    }
  }

  if (LED_PORTC_PIN_BM != 0) {
    PORTC = (PORTC & (~(LED_PORTC_PIN_BM))) | timesliceC[bitpos];
  }

  if (LED_PORTD_PIN_BM != 0) {
    PORTD = (PORTD & (~(LED_PORTD_PIN_BM))) | timesliceD[bitpos];
  }
}
