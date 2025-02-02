
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>

#include "bcm_config_constants.hpp"
#include "bcm_constants.hpp"
#include "bcm_types.hpp"

/*************************************************************
      GLOBAL VARIABLES
**************************************************************/
typedef uint8_t leds_t;

/*************************************************************
        FUNCTION PROTOTYPES
**************************************************************/
void BCM_init(void);
void BCM_stop(void);
void BCM_encode(duty_t dutyCycle[], char portLetter, const uint8_t LEDPins[],
                uint8_t nrOfLEDs);
