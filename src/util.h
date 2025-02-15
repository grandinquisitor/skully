#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>

uint8_t gammaCorrect(uint8_t brightness);
void smoothInt(uint16_t sample, uint8_t bits, int32_t* filter);
int16_t getFilterValue(int32_t filter);
int32_t setFilterValue(int16_t value);
bool countDown(uint32_t* prevTime, uint16_t wait);
uint8_t decr(uint8_t val, uint8_t mod);
uint8_t decr(uint8_t val, uint8_t sub, uint8_t mod);
uint8_t bit_cache(uint8_t b);
void sort_descending(int16_t* a, int16_t* b, int16_t* c);

#endif // __UTIL_H__