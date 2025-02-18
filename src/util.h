#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>

uint8_t gamma_correct(uint8_t brightness);
void smooth_int(uint16_t sample, uint8_t bits, int32_t* filter);
int16_t get_filter_value(int32_t filter);
int32_t set_filter_value(int16_t value);
bool count_down(uint32_t* prevTime, uint16_t wait);
uint8_t mod_decrement(uint8_t val, uint8_t mod);
uint8_t mod_subtract(uint8_t val, uint8_t sub, uint8_t mod);
uint8_t bit_mask_cache(uint8_t b);
void sort_descending(int16_t* a, int16_t* b, int16_t* c);
uint16_t approx_hypot(int16_t x, int16_t y, int16_t z);
uint16_t approx_hypot(int16_t x, int16_t y);

#endif // __UTIL_H__