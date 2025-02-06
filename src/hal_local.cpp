#include "hal_local.h"

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>

#include <Adafruit_LIS3DH.h>
#include <Wire.h>

#include <LowPower.h>

// only used for constants
#include <SparkFunLIS3DH.h>

#include "bcm.hpp"

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

constexpr uint8_t INT_PIN_BM = bit(PORTD2);

bool no_accel = false;

bool initAccel() {
  if (!lis.begin(ACCEL_ADDRESS)) {
    no_accel = true;
    return false;
  } else {
    lis.setRange(LIS3DH_RANGE_2_G);  // 2, 4, 8 or 16 G!
    lis.setClick(2, CLICKTHRESHHOLD);
    disableHpf();
    no_accel = false;
    return true;
  }
}

void init_mcu() {
  // don't need ADC
  ADCSRA = 0;

  // don't need analog comparator
  ACSR = (1 << ACD);

  // PRR = (1 << PRADC)   // ADC (already disabled by ADCSRA)
  //    | (1 << PRUSART0)  // USART0 (not used)
  //    | (1 << PRSPI)     // SPI (not used)
  //    | (1 << PRTIM1)    // Timer1 (not used)
  //    | (1 << PRTIM2);    // Timer2 (not used)

  // pinMode(2, INPUT_PULLUP);
  DDRD &= ~INT_PIN_BM;
  PORTD |= INT_PIN_BM;
}

void update_accel() {
  if (no_accel) {
    return;
  }
  lis.read();
}

int16_t getX() { return lis.x; }

int16_t getY() { return lis.y; }

bool got_click() {
  uint8_t click = lis.getClick();
  return !(click == 0 || !(click & 0x30));
}

void enable_output() {
  // all pins are connected to the anodes of leds
  // so setting high output means off
  PORTB |= LED_PORTB_PIN_BM;
  PORTC |= LED_PORTC_PIN_BM;
  PORTD |= LED_PORTD_PIN_BM;
  DDRB |= LED_PORTB_PIN_BM;
  DDRC |= LED_PORTC_PIN_BM;
  DDRD |= LED_PORTD_PIN_BM;
}

void disable_output() {
  // set all outputs to INPUT_PULLUP
  // all pins are connected to the anodes of leds
  // so setting output high might also work
  PORTB |= LED_PORTB_PIN_BM;
  PORTC |= LED_PORTC_PIN_BM;
  PORTD |= LED_PORTD_PIN_BM;
  DDRB &= ~LED_PORTB_PIN_BM;
  DDRC &= ~LED_PORTC_PIN_BM;
  DDRD &= ~LED_PORTD_PIN_BM;
}

void wakeUp() {
  // Just a handler for the pin interrupt.
  // might want this:
  // clearInterrupt();
}

void goToSleep() {
  BCM_stop();
  disable_output();
  configInterrupts();  // only really needs to be called once

  attachInterrupt(0, wakeUp, LOW);
  configInterrupts();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);

  disableHpf();
  enable_output();
  BCM_init();
}

// TODO: use the constants from the library
// from https://forums.adafruit.com/viewtopic.php?f=19&t=87936
void configInterrupts() {
  // configurations for control registers
  sendToAccel(0x20,
              0x57);  // Write 57h into CTRL_REG1;      // Turn on the sensor,
                      // enable X, Y, Z axes with ODR = 100Hz normal mode.
  sendToAccel(0x21, 0x09);  // Write 09h into CTRL_REG2;      // High-pass
                            // filter (HPF) enabled
  sendToAccel(0x22, 0x40);  // Write 40h into CTRL_REG3;      // ACC AOI1
                            // interrupt signal is routed to INT1 pin.
  sendToAccel(0x23,
              0x00);  // Write 00h into CTRL_REG4;      // Full Scale = +/-2 g
  sendToAccel(
      0x24,
      0x00);  // Write 08h into CTRL_REG5;      // Default value is 00 for no
              // latching. Interrupt signals on INT1 pin is not latched.
  // Users don’t need to read the INT1_SRC register to clear the interrupt
  // signal.
  sendToAccel(0x25, 0x02);  // supposed to invert the signal

  // configurations for wakeup and motionless detection
  sendToAccel(0x32, 0x10);  // Write 10h into INT1_THS;          // Threshold
                            // (THS) = 16LSBs * 15.625mg/LSB = 250mg.
  sendToAccel(0x33, 0x00);  // Write 00h into INT1_DURATION;     // Duration =
                            // 1LSBs * (1/10Hz) = 0.1s.
  // readRegister();  //Dummy read to force the HP filter to set reference
  // acceleration/tilt value
  sendToAccel(0x30, 0x2A);  // Write 2Ah into INT1_CFG;          // Enable XLIE,
                            // YLIE, ZLIE interrupt generation, OR logic.
}

void sendToAccel(uint8_t addr, uint8_t val) {
  Wire.beginTransmission(ACCEL_ADDRESS);  // transmit to device #44 (0x2c)
  // device address is specified in datasheet
  Wire.write(byte(addr));  // sends instruction byte
  Wire.write(val);         // sends potentiometer value byte
  Wire.endTransmission();  // stop transmitting
}

void clearInterrupt() {
  uint8_t dataRead;
  readFromAccel(LIS3DH_INT1_SRC, &dataRead);
}

void readFromAccel(uint8_t addr, uint8_t* output) {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(addr);
  if (Wire.endTransmission() != 0) {
    *output = 0;
  }
  Wire.requestFrom(ACCEL_ADDRESS, (uint8_t) 1);
  while (Wire.available())  // slave may send less than requested
  {
    *output = Wire.read();  // receive a byte as a proper uint8_t
  }
}

void disableHpf() { sendToAccel(0x21, 0x00); }