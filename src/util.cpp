#include "util.h"

#include <avr/pgmspace.h>
#include <Arduino.h>
#include <stdint.h>

#define USE_PROGMEM false

static constexpr uint8_t
#if USE_PROGMEM
    PROGMEM
#endif
        GAMMA8[] = {
            0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
            0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
            1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,
            3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,
            6,   6,   6,   6,   7,   7,   7,   7,   8,   8,   8,   9,   9,
            9,   10,  10,  10,  11,  11,  11,  12,  12,  13,  13,  13,  14,
            14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,
            21,  21,  22,  22,  23,  24,  24,  25,  25,  26,  27,  27,  28,
            29,  29,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,
            39,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,
            50,  51,  52,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
            64,  66,  67,  68,  69,  70,  72,  73,  74,  75,  77,  78,  79,
            81,  82,  83,  85,  86,  87,  89,  90,  92,  93,  95,  96,  98,
            99,  101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119,
            120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
            144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169,
            171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
            200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231,
            233, 236, 239, 241, 244, 247, 249, 252, 255};

uint8_t gamma_correct(uint8_t brightness) {
#if USE_PROGMEM
  return pgm_read_byte(&gamma8[brightness]);
#else
  return GAMMA8[brightness];
#endif
}

// bits should be log2 of period
void smooth_int(uint16_t sample, uint8_t bits, int32_t *filter) {
  int32_t local_sample = ((int32_t)sample) << 16;

  *filter += (local_sample - *filter) >> bits;
}

int16_t get_filter_value(int32_t filter) {
  return (int16_t)((filter + 0x8000) >> 16);
}

int32_t set_filter_value(int16_t value) { return ((int32_t)value) << 16; }

/**
 * Counts down from a specified wait time and updates the previous time reference.
 *
 * This function checks if the elapsed time since `*prev_time` exceeds `wait`.
 * If it does, `*prev_time` is updated to the current time, and the function returns true.
 * Otherwise, it returns false.
 *
 * @param prev_time Pointer to the previous time reference (in milliseconds).
 * @param wait The wait time to count down (in milliseconds).
 * @return True if the wait time has elapsed, false otherwise.
 */
bool count_down(uint32_t *prev_time, uint16_t wait) {
  if (millis() - (*prev_time) > wait) {
    (*prev_time) = millis();
    return true;
  } else {
    return false;
  }
}

/**
 * Decrements a value within a modulo system, wrapping around if necessary.
 *
 * @param val The value to decrement.
 * @param mod The modulus value defining the range [0, mod - 1].
 * @return The result of (val - 1) modulo mod, wrapping around to mod - 1 if val
 * is 0.
 */
uint8_t mod_decrement(uint8_t val, uint8_t mod) {
  return (val == 0) ? mod - 1 : val - 1;
}

/**
 * Subtracts a value within a modulo system, wrapping around if necessary.
 *
 * @param val The value to subtract from.
 * @param sub The value to subtract.
 * @param mod The modulus value defining the range [0, mod - 1].
 * @return The result of (val - sub) modulo mod, wrapping around if val < sub.
 */
uint8_t mod_subtract(uint8_t val, uint8_t sub, uint8_t mod) {
  if (val < sub) {
    return mod - (sub - val);
  } else {
    return val - sub;
  }
}

/**
 * Sorts three integers in descending order in-place.
 *
 * The function ensures that the values pointed to by `a`, `b`, and `c` are
 * rearranged such that `*a >= *b >= *c` after execution.
 *
 * @param a Pointer to the first integer.
 * @param b Pointer to the second integer.
 * @param c Pointer to the third integer.
 */
void sort_descending(int16_t *a, int16_t *b, int16_t *c) {
  // Swap if necessary to ensure a >= b >= c
  if (*a < *b) {
    int16_t temp = *a;
    *a = *b;
    *b = temp;
  }
  if (*a < *c) {
    int16_t temp = *a;
    *a = *c;
    *c = temp;
  }
  if (*b < *c) {
    int16_t temp = *b;
    *b = *c;
    *c = temp;
  }
}

static constexpr uint8_t BITS8[8] = {_BV(0), _BV(1), _BV(2), _BV(3),
                                     _BV(4), _BV(5), _BV(6), _BV(7)};

/**
 * Retrieves a precomputed bitmask from a lookup table (LUT) for a given bit
 * position.
 *
 * This function is equivalent to `_BV()` but uses a lookup table for faster
 * computation on systems without a native shift-by-n instruction.
 *
 * @param bp The bit position (0-7) for which to retrieve the bitmask.
 * @return The bitmask corresponding to the specified bit position.
 */
uint8_t bit_mask_cache(uint8_t bp) { return BITS8[bp]; }

/**
 * Computes an approximate 3D hypotenuse (magnitude) of three integer values.
 *
 * This function calculates an approximate magnitude of the vector (x, y, z)
 * using a fast approximation method.
 *
 * @see https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
 * @note This is an extension of the 2D Alpha max plus beta min algorithm that
 * does not have a specific name in literature.
 * @param x The first component of the vector.
 * @param y The second component of the vector.
 * @param z The third component of the vector.
 * @return The approximate magnitude of the vector.
 */
uint16_t approx_hypot(int16_t x, int16_t y, int16_t z) {
  x = abs(x);
  y = abs(y);
  z = abs(z);

  sort_descending(&x, &y, &z);

  return max(x, (15 * x + 6 * y + 5 * z) >> 4);
}

/**
 * Computes an approximate 2D hypotenuse (magnitude) of three integer values.
 *
 * This function calculates an approximate magnitude of the vector (x, y) using
 * a fast approximation method.
 *
 * @see https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
 * @param x The first component of the vector.
 * @param y The second component of the vector.
 * @return The approximate magnitude of the vector.
 */
uint16_t approx_hypot(int16_t x, int16_t y) {
  x = abs(x);
  y = abs(y);

  if (x > y) {
    y >>= 2;
  } else {
    x >>= 2;
  }

  return x + y;
}