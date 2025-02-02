#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>

uint8_t gammaCorrect(uint8_t brightness);
void smoothInt(uint16_t sample, uint8_t bits, int32_t* filter);
int16_t getFilterValue(int32_t filter);
int32_t setFilterValue(int16_t value);
void smoothInt2(uint16_t sample, uint8_t bits, int32_t* filter, int32_t* filter2);
uint8_t decr(uint8_t val, uint8_t mod);
uint8_t decr(uint8_t val, uint8_t sub, uint8_t mod);

#endif // __UTIL_H__