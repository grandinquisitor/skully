#include "animation.h"

#include <stdint.h>

#include "FastLED.h"

#include "constants.h"
#include "util.h"

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

constexpr BlendMode BLEND_MODE = BlendMode::BLEND_87_5;


bool animation_update(uint8_t brightness[], uint16_t roll, bool click, bool reset) {
  static int8_t prevBottomLed = -1;
  static uint32_t directionRegister = 0;
  static uint8_t sparking = DEFAULT_SPARKING;

  static int32_t xFilter = 0;
  static int32_t yFilter = 0;

  uint8_t bottomLed = map(roll, 0, 0xffff, 0, NUM_LEDS);

  uint8_t topLed = addmod8(bottomLed, (NUM_LEDS / 2), NUM_LEDS);

  directionRegister = 0;
  for (uint8_t i = 0; i < NUM_LEDS / 2; ++i) {
    bitSet(directionRegister, addmod8(bottomLed, i, NUM_LEDS));
  }

  static uint32_t lastForcedSpark = 0;
  static uint32_t wokeAgo = 0;
  bool sparkNow = false;
  if (click || reset) {
    sparkNow = true;
    lastForcedSpark = millis();
    if (reset) {
      wokeAgo = 0;
    }
  } else if (millis() - lastForcedSpark < 400) {
    sparkNow = true;
  } else if (millis() - wokeAgo < 2500) {
    sparkNow = true;
  } else if (millis() - lastForcedSpark < 5000) {
    sparking = map(millis() - lastForcedSpark, 400, 5000, 120, 50);
    //    sparking = 120;
  } else if (millis() - lastForcedSpark < 10000) {
    sparking = map(millis() - lastForcedSpark, 5000, 10000, 50, 10);
    //    sparking = 50;
  } else {
    // sleep now
    return true;
  }


  // Array of temperature readings at each simulation cell
  static uint8_t heat[NUM_LEDS];


  // Step 1.  Cool down every cell a little
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    //    heat[i] = qsub8(heat[i],  random8(0, ((((uint16_t) lerp8by8(MIN_COOLING, MAX_COOLING, currentCooling)) * 10) / NUM_LEDS) + 2));
    heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // right side
  for (uint8_t k = 0; k < NUM_LEDS; ++k) {
    if (!bitRead(directionRegister, k)) {

      uint8_t neighbor1 = addmod8(k, 1, NUM_LEDS);
      neighbor1 = !bitRead(directionRegister, neighbor1) ? heat[neighbor1] : 0;

      uint8_t neighbor2 = addmod8(k, 2, NUM_LEDS);
      neighbor2 = !bitRead(directionRegister, neighbor2) ? heat[neighbor2] : 0;

      //      heat[k] = (((uint16_t) neighbor1) + neighbor2 + neighbor2) / 3;
      heat[k] = lerp8by8(heat[k], (((uint16_t)neighbor1) + neighbor2 + neighbor2) / 3, BACK_FADE);
    }
  }

  // left side
  for (int8_t k = NUM_LEDS - 1; k >= 0; --k) {
    if (bitRead(directionRegister, k)) {
      uint8_t neighbor1 = decr(k, 1, NUM_LEDS);
      neighbor1 = bitRead(directionRegister, neighbor1) ? heat[neighbor1] : 0;

      uint8_t neighbor2 = decr(k, 2, NUM_LEDS);
      neighbor2 = bitRead(directionRegister, neighbor2) ? heat[neighbor2] : 0;

      //      heat[k] = (((uint16_t) neighbor1) + neighbor2 + neighbor2) / 3;
      heat[k] = lerp8by8(heat[k], (((uint16_t)neighbor1) + neighbor2 + neighbor2) / 3, BACK_FADE);
    }
  }

  // to involve pitch: weighted random that diffusion goes [heat - 2] * 2 vs. [heat + 1] etc.

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if (sparkNow || random8() < sparking) {

    uint8_t y = random8(NUM_LEDS / 6);
    if (random8() <= 160) {
      y = addmod8(bottomLed, y, NUM_LEDS);
    } else {
      y = decr(bottomLed, y + 1, NUM_LEDS);
    }

    heat[y] = qadd8(heat[y], sparkNow ? 255 : random8(160, 255));
  }

  // Step 4.  Map from heat cells to LED colors
  for (int j = 0; j < NUM_LEDS; j++) {
    blendIt(BLEND_MODE, brightness[j], gammaCorrect(heat[j]));
  }
}


void update_led_pattern(uint8_t brightness[]) {
  static uint8_t phase = 0;
  static uint8_t current_led = 0;

  // Fade all LEDs
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (brightness[i] > 0) brightness[i]--;
  }

  // Create a new pattern
  switch (phase) {
    case 0:  // Single LED chase
      brightness[current_led] = 255;
      if (++current_led >= NUM_LEDS) {
        current_led = 0;
        phase = 1;
      }
      break;

    case 1:  // Alternating LEDs
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        brightness[i] = (i % 2 == current_led % 2) ? 255 : 0;
      }
      if (++current_led >= 4) {
        current_led = 0;
        phase = 2;
      }
      break;

    case 2:  // Fade in/out all LEDs
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        brightness[i] = current_led;
      }
      current_led += 5;
      if (current_led >= 255) {
        current_led = 0;
        phase = 0;
      }
      break;
  }
}

void blendIt(BlendMode blendMode, uint8_t& brightness_ref, uint8_t new_val) {
  switch (blendMode) {
    case BlendMode::BLEND_0:
      brightness_ref = new_val;
      break;
    case BlendMode::BLEND_50:
      brightness_ref = (brightness_ref >> 1) + (new_val >> 1);
      break;
    case BlendMode::BLEND_75:
      brightness_ref = (brightness_ref >> 2) + (new_val >> 1) + (new_val >> 2);
      break;
    case BlendMode::BLEND_87_5:
      brightness_ref = (brightness_ref >> 3) + (new_val >> 1) + (new_val >> 2) + (new_val >> 3);
      break;
  }
}