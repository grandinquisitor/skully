#ifndef __BCM_CONFIG_CONSTANTS_H__
#define __BCM_CONFIG_CONSTANTS_H__

#include <stdbool.h>
#include <stdint.h>

#include "constants.h"
#include "bcm_types.hpp"


#define MICROS_TIMER MICROS_TIMER1  // Which timer to use
#define PRESCALER 256
#define NUMBER_OF_BITS BITS_8
#define USE_LOGARITHMIC_ARRAY FALSE

#if NUMBER_OF_BITS == BITS_8
typedef uint8_t duty_t;
#else
typedef uint16_t duty_t;
#endif

// Which ports are used
#define LEDS_ON_PINS_PORTB port_b_bm
#define LEDS_ON_PINS_PORTC port_c_bm
#define LEDS_ON_PINS_PORTD port_d_bm
#define LEDS_ON_PORT_E 0

constexpr bool INVERT_POLARITY = true;

#endif // __BCM_CONFIG_CONSTANTS_H__