#include "util.h"

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#define USE_PROGMEM 1

constexpr uint8_t
#if USE_PROGMEM
    PROGMEM
#endif
        gamma8[] = {
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

uint8_t gammaCorrect(uint8_t brightness) {
#if USE_PROGMEM
  return pgm_read_byte(&gamma8[brightness]);
#else
  return gamma8[brightness];
#endif
}

// bits should be log2 of period
void smoothInt(uint16_t sample, uint8_t bits, int32_t *filter) {
  long local_sample = ((long)sample) << 16;

  *filter += (local_sample - *filter) >> bits;
}

int16_t getFilterValue(int32_t filter) {
  return (int16_t)((filter + 0x8000) >> 16);
}

int32_t setFilterValue(int16_t value) {
  return ((int32_t)value) << 16;
}

bool countDown(uint32_t *prevTime, uint16_t wait) {
  if (millis() - (*prevTime) > wait) {
    (*prevTime) = millis();
    return true;
  } else {
    return false;
  }
}

void smoothInt2(uint16_t sample, uint8_t bits, int32_t *filter, int32_t *filter2) {
  int32_t local_sample = ((int32_t)sample) << 16;
  int32_t delta = local_sample - (*filter);
  *filter2 += (delta - *filter2) >> bits;
  smoothInt(sample, bits, filter);
}

uint8_t decr(uint8_t val, uint8_t mod) {
  return (val == 0) ? mod - 1 : val - 1;
}

uint8_t decr(uint8_t val, uint8_t sub, uint8_t mod) {
  if (val < sub) {
    return mod - (sub - val);
  } else {
    return val - sub;
  }
}
