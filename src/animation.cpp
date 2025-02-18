#include "animation.h"

#include <FastLED.h>
#include <stdint.h>

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

bool calculate_next_frame(uint8_t brightness[], uint16_t roll, bool click,
                          bool reset) {
  static uint32_t s_direction_register = 0;
  static uint8_t s_sparking = DEFAULT_SPARKING;

  uint8_t bottom_led = map(roll, 0, 0xffff, 0, NUM_LEDS);
  // uint8_t topLed = addmod8(bottom_led, (NUM_LEDS / 2), NUM_LEDS);

  s_direction_register = 0;
  for (uint8_t i = 0; i < NUM_LEDS / 2; ++i) {
    bitSet(s_direction_register, addmod8(bottom_led, i, NUM_LEDS));
  }

  static uint32_t last_forced_spark = 0;
  static uint32_t woke_ago = 0;
  constexpr uint32_t WOKE_AGO_TIMEOUT = 2500;
  constexpr uint32_t FORCED_SPARK_LEVEL1 = 400 * 6;
  constexpr uint32_t FORCED_SPARK_LEVEL2 = 5000 * 6;
  constexpr uint32_t FORCED_SPARK_LEVEL3 = 60000;
  bool spark_now = false;
  uint32_t now = millis();
  if (click || reset) {
    spark_now = true;
    last_forced_spark = now;
    if (reset) {
      woke_ago = 0;
    }
  } else if (now - last_forced_spark < FORCED_SPARK_LEVEL1) {
    spark_now = true;
  } else if (now - woke_ago < WOKE_AGO_TIMEOUT) {
    spark_now = true;
  } else if (now - last_forced_spark < FORCED_SPARK_LEVEL2) {
    s_sparking = map(now - last_forced_spark, FORCED_SPARK_LEVEL1,
                   FORCED_SPARK_LEVEL2, 120, 50);
  } else if (now - last_forced_spark < FORCED_SPARK_LEVEL3) {
    s_sparking = map(now - last_forced_spark, FORCED_SPARK_LEVEL2,
                   FORCED_SPARK_LEVEL3, 50, 10);
  } else {
    // sleep now
    return true;
  }

  // Array of temperature readings at each simulation cell
  static uint8_t s_heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    //    s_heat[i] = qsub8(s_heat[i],  random8(0, ((((uint16_t)
    //    lerp8by8(MIN_COOLING, MAX_COOLING, currentCooling)) * 10) / NUM_LEDS)
    //    + 2));
    s_heat[i] = qsub8(s_heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // right side
  for (uint8_t k = 0; k < NUM_LEDS; ++k) {
    if (!bitRead(s_direction_register, k)) {
      uint8_t neighbor1 = addmod8(k, 1, NUM_LEDS);
      neighbor1 = !bitRead(s_direction_register, neighbor1) ? s_heat[neighbor1] : 0;

      uint8_t neighbor2 = addmod8(k, 2, NUM_LEDS);
      neighbor2 = !bitRead(s_direction_register, neighbor2) ? s_heat[neighbor2] : 0;

      //      heat[k] = (((uint16_t) neighbor1) + neighbor2 + neighbor2) / 3;
      s_heat[k] =
          lerp8by8(s_heat[k], (((uint16_t)neighbor1) + neighbor2 + neighbor2) / 3,
                   BACK_FADE);
    }
  }

  // left side
  for (int8_t k = NUM_LEDS - 1; k >= 0; --k) {
    if (bitRead(s_direction_register, k)) {
      uint8_t neighbor1 = mod_subtract(k, 1, NUM_LEDS);
      neighbor1 = bitRead(s_direction_register, neighbor1) ? s_heat[neighbor1] : 0;

      uint8_t neighbor2 = mod_subtract(k, 2, NUM_LEDS);
      neighbor2 = bitRead(s_direction_register, neighbor2) ? s_heat[neighbor2] : 0;

      //      heat[k] = (((uint16_t) neighbor1) + neighbor2 + neighbor2) / 3;
      s_heat[k] =
          lerp8by8(s_heat[k], (((uint16_t)neighbor1) + neighbor2 + neighbor2) / 3,
                   BACK_FADE);
    }
  }

  // to involve pitch: weighted random that diffusion goes [s_heat - 2] * 2 vs.
  // [s_heat + 1] etc.

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if (spark_now || random8() < s_sparking) {
    uint8_t y = random8(NUM_LEDS / 6);
    if (random8() <= 160) {
      y = addmod8(bottom_led, y, NUM_LEDS);
    } else {
      y = mod_subtract(bottom_led, y + 1, NUM_LEDS);
    }

    s_heat[y] = qadd8(s_heat[y], spark_now ? 255 : random8(160, 255));
  }

  // Step 4.  Map from heat cells to LED colors
  for (int j = 0; j < NUM_LEDS; j++) {
    blendIt(BLEND_MODE, brightness[j], gamma_correct(s_heat[j]));
  }

  return false;
}

void update_led_pattern(uint8_t brightness[]) {
  static uint8_t s_phase = 0;
  static uint8_t s_current_led = 0;

  // Fade all LEDs
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (brightness[i] > 0) brightness[i]--;
  }

  // Create a new pattern
  switch (s_phase) {
    case 0:  // Single LED chase
      brightness[s_current_led] = 255;
      if (++s_current_led >= NUM_LEDS) {
        s_current_led = 0;
        s_phase = 1;
      }
      break;

    case 1:  // Alternating LEDs
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        brightness[i] = (i % 2 == s_current_led % 2) ? 255 : 0;
      }
      if (++s_current_led >= 4) {
        s_current_led = 0;
        s_phase = 2;
      }
      break;

    case 2:  // Fade in/out all LEDs
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        brightness[i] = s_current_led;
      }
      s_current_led += 5;
      if (s_current_led >= 255) {
        s_current_led = 0;
        s_phase = 0;
      }
      break;
  }
}

void blendIt(BlendMode blend_mode, uint8_t& brightness_ref, uint8_t new_val) {
  switch (blend_mode) {
    case BlendMode::BLEND_0:
      brightness_ref = new_val;
      break;
    case BlendMode::BLEND_25:
      brightness_ref = (brightness_ref >> 1) + (brightness_ref >> 2) + (new_val >> 2);
      break;
    case BlendMode::BLEND_50:
      brightness_ref = (brightness_ref >> 1) + (new_val >> 1);
      break;
    case BlendMode::BLEND_75:
      brightness_ref = (brightness_ref >> 2) + (new_val >> 1) + (new_val >> 2);
      break;
    case BlendMode::BLEND_87_5:
      brightness_ref = (brightness_ref >> 3) + (new_val >> 1) + (new_val >> 2) +
                       (new_val >> 3);
      break;
  }
}

// test whether the accelerometer works
bool angle_test_animation(uint8_t brightness[], uint16_t roll, bool click,
                          bool reset) {
  uint8_t bottom_led = map(roll, 0, 0xffff, 0, NUM_LEDS);

  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    if (i != bottom_led) {
      if (click) {
        brightness[i] = 1;
      } else {
        brightness[i] = 0;
      }
    } else {
      brightness[i] = 64;
    }
  }

  return false;
}