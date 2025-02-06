#include <Arduino.h>

#include "FastLED.h"

#include "bcm.hpp"
#include "hal_local.h"
#include "constants.h"
#include "fxpt_atan2.h"
#include "util.h"

void setup() {
  init_mcu();
  initAccel();
  BCM_init(); // note calls sei();
}

void loop() {

  // static int8_t prevBottomLed = -1;
  static uint32_t directionRegister = 0;

  static int32_t xFilter = 0;
  static int32_t yFilter = 0;

  static uint8_t sparking = DEFAULT_SPARKING;

  update_accel();


  if (xFilter == 0 && yFilter == 0) {
    xFilter = setFilterValue(getX());
    yFilter = setFilterValue(getY());
  } else {
    smoothInt(getX(), 1, &xFilter);
    smoothInt(getY(), 1, &yFilter);
  }

  uint16_t roll = fxpt_atan2(getFilterValue(xFilter), getFilterValue(yFilter)) + ANGLE_OFFSET;

  uint8_t bottomLed = map(roll, 0, 0xffff, 0, NUM_LEDS);

  // uint8_t topLed = addmod8(bottomLed, (NUM_LEDS / 2), NUM_LEDS);

  directionRegister = 0;
  for (uint8_t i = 0; i < NUM_LEDS / 2; ++i) {
    bitSet(directionRegister, addmod8(bottomLed, i, NUM_LEDS));
  }

  static uint32_t lastForcedSpark = 0;
  static uint32_t wokeAgo = 0;
  bool sparkNow = false;
  if (got_click()) {
    sparkNow = true;
    lastForcedSpark = millis();
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
    goToSleep();
    sparkNow = true;
    wokeAgo = millis();
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
    brightnesses[led_to_map[j]] = gammaCorrect(heat[j]);
  }

  // Encode brightnesses for each port
  // cli(); // might need
  BCM_encode(brightnesses, 'B', LED_PORTB_PINS, NUM_LED_PORTB_PINS);                            // PORTB
  BCM_encode(brightnesses + NUM_LED_PORTB_PINS, 'C', LED_PORTC_PINS, NUM_LED_PORTC_PINS);               // PORTC
  BCM_encode(brightnesses + NUM_LED_PORTB_PINS + NUM_LED_PORTC_PINS, 'D', LED_PORTD_PINS, NUM_LED_PORTD_PINS);  // PORTD
                                                                                // sei(); // might need

  // TODO: set this to a timer, could idle sleep in between
  // unsigned long const WAIT = 500000 / Palatis::SoftPWM.PWMlevels() / 2;
  // unsigned long nextMicros = 0;
  // while (micros() < nextMicros)
  //   ;
  // nextMicros = micros() + WAIT;
  // 976.5625µs = ~1ms
  delay(1);
}