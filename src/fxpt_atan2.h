#ifndef __FXPT_ATAN2_H__
#define __FXPT_ATAN2_H__

#include <stdint.h>

int32_t q15_mul(const int32_t j, const int32_t k);
int32_t q15_div(const int32_t numer, const int32_t denom);
int16_t fxpt_atan2(const int32_t y, const int32_t x);

#endif // __FXPT_ATAN2_H__