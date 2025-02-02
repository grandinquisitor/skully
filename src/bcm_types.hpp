#ifndef __BCM_TYPES_H__
#define __BCM_TYPES_H__

#ifndef PORTE
#define PINE _SFR_IO8(0x0C)
#define DDRE _SFR_IO8(0x0D)
#define PORTE _SFR_IO8(0x0E)
#endif

#define BITS_8 8
#define BITS_9 9
#define BITS_10 10

#define LED_LIMIT_8 uint8_t    // up to 8 leds
#define LED_LIMIT_16 uint16_t  // up to 16 leds
#define LED_LIMIT_32 uint32_t  // up to 32 leds

#define MICROS_TIMER0 0  // Use timer0
#define MICROS_TIMER1 1  // Use timer1
#define MICROS_TIMER2 2  // Use timer2
#endif // __BCM_TYPES_H__