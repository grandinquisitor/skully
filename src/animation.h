#ifndef __ANIMATION_H__
#define __ANIMATION_H__

#include <stdint.h>


enum class BlendMode {
  BLEND_0,
  BLEND_50,
  BLEND_75,
  BLEND_87_5
};


void blendIt(BlendMode blendMode, uint8_t& brightness_ref, uint8_t new_val);

#endif // __ANIMATION_H__