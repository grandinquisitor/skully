#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <stdbool.h>
#include <stdint.h>

// -- LED CONSTANTS --
constexpr uint8_t NUM_LEDS = 18;

extern uint8_t brightnesses[NUM_LEDS];

//  = {
//     0,  // PORTB0,
//     0,  // PORTB1,
//     0,  // PORTB2,
//     0,  // PORTB3,
//     0,  // PORTB4,
//     0,  // PORTB5,
//     0,  // PORTB6,
//     0,  // PORTB7,
//     0,  // PORTC0,
//     0,  // PORTC1,
//     0,  // PORTC2,
//     0,  // PORTC3,
//     0,  // PORTD0,
//     0,  // PORTD3,
//     0,  // PORTD4,
//     0,  // PORTD5,
//     0,  // PORTD6,
//     0,  // PORTD7,
// };

constexpr uint8_t led_to_map[NUM_LEDS] = {
    12,  // PORTD0
    9,   // PORTC1
    8,   // PORTC0
    5,   // PORTB5
    4,   // PORTB4
    3,   // PORTB3
    2,   // PORTB2
    1,   // PORTB1
    0,   // PORTB0
    17,  // PORTD7
    16,  // PORTD6
    15,  // PORTD5
    7,   // PORTB7
    6,   // PORTB6
    14,  // PORTD4
    13,  // PORTD3
    11,  // PORTC3
    10,  // PORTC2
};

// optimize this with bit masks
constexpr uint8_t num_b_pins = 8;
constexpr uint8_t num_c_pins = 4;
constexpr uint8_t num_d_pins = 6;
constexpr uint8_t b_pins[num_b_pins] = {0, 1, 2, 3, 4, 5, 6, 7};
constexpr uint8_t c_pins[num_c_pins] = {0, 1, 2, 3};
constexpr uint8_t d_pins[num_d_pins] = {0, 3, 4, 5, 6, 7};

constexpr uint8_t make_bitmap(const uint8_t* arr, uint8_t len) {
  uint8_t bm = 0;
  for (uint8_t i = 0; i < len; ++i) {
    bm |= 1 << arr[i];
  }
  return bm;
}

constexpr uint8_t port_b_bm = make_bitmap(b_pins, num_b_pins);
constexpr uint8_t port_c_bm = make_bitmap(c_pins, num_c_pins);
constexpr uint8_t port_d_bm = make_bitmap(d_pins, num_d_pins);

/// -- FIRE CONSTANTS --

// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100
constexpr uint8_t COOLING = 55;

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
constexpr uint8_t DEFAULT_SPARKING = 50;
// good for normal state = 120

constexpr uint8_t BACK_FADE = 230;
constexpr uint8_t CLICKTHRESHHOLD = 20;

// -- HAL CONSTANTS --
constexpr uint8_t ACCEL_ADDRESS = 0x19;

constexpr uint16_t ANGLE_OFFSET = 0x3fff;  // based on the orientation of the chip on the board

#endif // __CONSTANTS_H__