#include <avr/io.h>
#include <avr/interrupt.h>

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <LowPower.h>
#include <FastLED.h>

#include "fxpt_atan2.h"
#include "lis3dh_reg.h"
#include "animation.h"
#include "util.h"

// Calculate update interval based on timer configuration
// Timer2 frequency = F_CPU / (prescaler * (1 + OCR2A))
// For 8ms update: 8ms = 256 * prescaler * (1 + 1) / F_CPU
// With prescaler = 32: Updates per tick = 256 / (F_CPU * 0.008)
// (256 is an approximation)
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#define TIMER_PRESCALER 32
#define DESIRED_UPDATE_MS 45
#define UPDATES_PER_ANIMATION ((F_CPU / 1000 * DESIRED_UPDATE_MS) / (256 * TIMER_PRESCALER))

#if UPDATES_PER_ANIMATION == 0
#error "update rate is < 1"
#endif

#define TIMER_USED 2

typedef enum {
  PORT_B = 0,
  PORT_C = 1,
  PORT_D = 2
} PortIndex;

typedef struct {
  PortIndex port;
  uint8_t pin;
} LedMapping;

constexpr uint8_t NUM_LEDS = 18;
constexpr LedMapping LED_MAP[NUM_LEDS] = {
  { PORT_D, 0 },  // PORTD0
  { PORT_C, 1 },  // PORTC1
  { PORT_C, 0 },  // PORTC0
  { PORT_B, 5 },  // PORTB5
  { PORT_B, 4 },  // PORTB4
  { PORT_B, 3 },  // PORTB3
  { PORT_B, 2 },  // PORTB2
  { PORT_B, 1 },  // PORTB1
  { PORT_B, 0 },  // PORTB0
  { PORT_D, 7 },  // PORTD7
  { PORT_D, 6 },  // PORTD6
  { PORT_D, 5 },  // PORTD5
  { PORT_B, 7 },  // PORTB7
  { PORT_B, 6 },  // PORTB6
  { PORT_D, 4 },  // PORTD4
  { PORT_D, 3 },  // PORTD3
  { PORT_C, 3 },  // PORTC3
  { PORT_C, 2 }   // PORTC2
};

constexpr uint8_t _make_bitmap(PortIndex pi) {
  uint8_t bm = 0;
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    if (LED_MAP[i].port == pi) {
      bm |= _BV(LED_MAP[i].pin);
    }
  }
  return bm;
}

constexpr uint8_t PORT_B_MASK = _make_bitmap(PORT_B);
constexpr uint8_t PORT_C_MASK = _make_bitmap(PORT_C);
constexpr uint8_t PORT_D_MASK = _make_bitmap(PORT_D);

// Global variables
// so I can use memset() to reset all of them at once
// volatile uint8_t g_timeslice[3][8] = {0};
// volatile uint8_t *g_timeslice_b = g_timeslice[0];
// volatile uint8_t *g_timeslice_c = g_timeslice[1];
// volatile uint8_t *g_timeslice_d = g_timeslice[2];

volatile uint8_t g_timeslice_b[8];
volatile uint8_t g_timeslice_c[8];
volatile uint8_t g_timeslice_d[8];
volatile uint8_t g_tick = 0;
volatile uint8_t g_bitpos = 0;
volatile uint8_t brightness[NUM_LEDS] = { 0 };


Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// set to false to use LIS
// if false and LIS setup fails, setup will overwrite this to true
bool NO_LIS = false;

constexpr uint8_t CLICKTHRESHHOLD = 20;
constexpr uint8_t ACCEL_ADDRESS = 0x19;


// void led_init(void);
// void led_encode_timeslices(uint8_t intensity[]);
// void update_led_pattern(uint8_t brightness[]);
// void power_config();

__attribute((OS_main)) int main(void) {
  uint32_t animation_timestamp = 0;

  led_init();

  // Initialize all brightness values to 0
  // memset()
  // for (uint8_t i = 0; i < NUM_LEDS; i++) {
  //   brightness[i] = 0;
  // }

  led_encode_timeslices(brightness);

  other_hardware_config();

  if (!NO_LIS) {
    Wire.begin();

    if (!connect_accel()) {
      showError();
    }
  }

  sei();

  bool animation_reset = true;
  bool accel_reset = true;

  while (1) {
    while (g_tick == 0) { /*wait for g_tick to be non-zero*/
    }
    g_tick = 0;  //consume the tick

    // Update animation based on calculated interval
    if (++animation_timestamp >= UPDATES_PER_ANIMATION) {
      animation_timestamp = 0;
      bool animation_off = animation_update(brightness, getAngle(accel_reset), getClick(), animation_reset);
      animation_reset = false;
      accel_reset = false;

      if (animation_off && !NO_LIS) {
        // goToSleep();
        animation_reset = true;
      }
    }

    led_encode_timeslices(brightness);
  }

  return 0;
}

void led_init(void) {
  // Initialize all ports to HIGH, turn LEDs off (LEDs are active LOW)
  PORTB |= PORT_B_MASK;
  PORTC |= PORT_C_MASK;
  PORTD |= PORT_D_MASK;

  // Set required pins as outputs
  DDRB |= PORT_B_MASK;
  DDRC |= PORT_C_MASK;
  DDRD |= PORT_D_MASK;

  setup_timer();
}


void stop_leds(void) {
  stop_timer();
  // turn all leds off, set to INPUT_PULLUP
  PORTB |= PORT_B_MASK;
  PORTC |= PORT_C_MASK;
  PORTD |= PORT_D_MASK;

  DDRB &= ~PORT_B_MASK;
  DDRC &= ~PORT_C_MASK;
  DDRD &= ~PORT_D_MASK;
}


void setup_timer() {

#if TIMER_USED != 2
#error "selected timer not implemented"
#endif

  // Configure Timer2 for PWM
  TCCR2A = (1 << WGM21);  // CTC mode

#if TIMER_PRESCALER == 1
  TCCR2B = _BV(CS20);  // Prescaler 1
#elif TIMER_PRESCALER == 8
  TCCR2B = _BV(CS21);  // Prescaler 8
#elif TIMER_PRESCALER == 32
  TCCR2B = _BV(CS21) | _BV(CS20);  // Prescaler 32
#elif TIMER_PRESCALER == 64
  TCCR2B = _BV(CS22);  // Prescaler 64
#elif TIMER_PRESCALER == 128
  TCCR2B = _BV(CS22) | _BV(CS20);  // Prescaler 128
#elif TIMER_PRESCALER == 256
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler 256
#elif TIMER_PRESCALER == 1024
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);  // Prescaler 1024
#else
#error "Unsupported TIMER_PRESCALER value"
#endif

  OCR2A = 1;
  TIMSK2 = (1 << OCIE2A);  // Enable compare match interrupt
}

void stop_timer() {
  // assume cli() has been called?

  // stops the ISR
  TIMSK2 &= ~_BV(OCIE2A);

  // stops the timer clock -- probably not strictly necessary
  // TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));

  // reset the timer/counter state (not strictly needed)
  // TCNT2 = 0;  // Reset the timer counter
  // OCR2A = 0;  // Reset the output compare register
}

void other_hardware_config() {
  // don't need ADC
  ADCSRA = 0;

  // don't need analog comparator
  ACSR = _BV(ACD);

  PRR = _BV(PRADC)       // ADC (already disabled by ADCSRA)
        | _BV(PRUSART0)  // USART0 (not used)
        | _BV(PRSPI)     // SPI (not used)
#if TIMER_USED == 2
        | _BV(PRTIM1)  // Timer1
#elif TIMER_USED == 1
        | _BV(PRTIM2)  // Timer2
#else
#error "unexpected timer used"
#endif
    ;
  // assume timer0 is doing millis()

  // should be true anyway
  // sleep_bod_disable();

  // pinMode(2, INPUT_PULLUP);
  constexpr uint8_t INT_PIN_BM = _BV(PORTD2);
  DDRD &= ~INT_PIN_BM;
  PORTD |= INT_PIN_BM;
}

void led_encode_timeslices(volatile uint8_t intensity[]) {
  // Clear all timeslices (start with all LEDs off)

  // memset(g_timeslice_b, 0xFF, 8);
  for (uint8_t bitpos = 0; bitpos < 8; bitpos++) {
    g_timeslice_b[bitpos] = 0xFF;
    g_timeslice_c[bitpos] = 0xFF;
    g_timeslice_d[bitpos] = 0xFF;
  }

  // For each LED
  for (uint8_t led = 0; led < NUM_LEDS; led++) {
    // For each bit position in the brightness value
    for (uint8_t bitpos = 0, bitmask = 1; bitpos < 8; bitpos++, bitmask <<= 1) {
      if (intensity[led] & bitmask) {
        // If this bit is set, turn on the LED for this timeslice
        // TODO: LUT this bit shift
        uint8_t pin_mask = ~_BV(LED_MAP[led].pin);  // Inverted because LOW = ON

        // g_timeslice[LED_MAP[led].port][bitpos] &= pin_mask;
        switch (LED_MAP[led].port) {
          case PORT_B:
            g_timeslice_b[bitpos] &= pin_mask;
            break;
          case PORT_C:
            g_timeslice_c[bitpos] &= pin_mask;
            break;
          case PORT_D:
            g_timeslice_d[bitpos] &= pin_mask;
            break;
        }
      }
    }
  }
}

// ---
// accelerometer/I2C functions
bool connect_accel() {
  if (!lis.begin(ACCEL_ADDRESS)) {
    NO_LIS = true;
    return false;
  } else {
    NO_LIS = false;
    // TODO: overwritten by configInterrupts anyway, rewrite using a consistent driver
    lis.setRange(LIS3DH_RANGE_2_G);  // 2, 4, 8 or 16 G!
    lis.setClick(2, CLICKTHRESHHOLD);
    disableHpf();
    return true;
  }
}

void showError() {
  uint8_t old_ddrd4_bit = DDRD & _BV(DDD4);
  DDRD |= _BV(DDD4);

  for (uint8_t i = 0; i < 5; i++) {
    PORTD &= ~_BV(PORTD4);
    delay(500);
    PORTD |= _BV(PORTD4);
    delay(500);
  }

  if (old_ddrd4_bit) {
    DDRD |= _BV(DDD4);   // If original bit was 1, set it back to 1
  } else {
    DDRD &= ~_BV(DDD4);  // If original bit was 0, set it back to 0
  }
}

uint16_t getAngle(bool reset) {
  if (NO_LIS) {
    return 0;
  }

  constexpr uint16_t ACCEL_ANGLE_OFFSET = 0x3fff;  // based on the orientation of the chip on the board

  static int32_t xFilter, yFilter;

  lis.read();

  if (reset) {
    xFilter = setFilterValue(lis.x);
    yFilter = setFilterValue(lis.y);
  } else {
    smoothInt(lis.x, 1, &xFilter);
    smoothInt(lis.y, 1, &yFilter);
  }

  uint16_t roll = fxpt_atan2(getFilterValue(xFilter), getFilterValue(yFilter)) + ACCEL_ANGLE_OFFSET;
}

bool getClick() {
  if (NO_LIS) {
    return true;
  }
  uint8_t click = lis.getClick();
  return !(click == 0 || !(click & 0x30));
}

// TODO: replace with proper constants and use lis3dh_reg_t for readability
// from https://forums.adafruit.com/viewtopic.php?f=19&t=87936
void configInterrupts() {
  // configurations for control registers
  sendToAccel(0x20, 0x57);  //Write 57h into CTRL_REG1;      // Turn on the sensor, enable X, Y, Z axes with ODR = 100Hz normal mode.
  sendToAccel(0x21, 0x09);  //Write 09h into CTRL_REG2;      // High-pass filter (HPF) enabled
  sendToAccel(0x22, 0x40);  //Write 40h into CTRL_REG3;      // ACC AOI1 interrupt signal is routed to INT1 pin.
  sendToAccel(0x23, 0x00);  //Write 00h into CTRL_REG4;      // Full Scale = +/-2 g
  sendToAccel(0x24, 0x00);  //Write 08h into CTRL_REG5;      // Default value is 00 for no latching. Interrupt signals on INT1 pin is not latched.
  //Users don’t need to read the INT1_SRC register to clear the interrupt signal.
  sendToAccel(0x25, 0x02);  // supposed to invert the signal

  // configurations for wakeup and motionless detection
  sendToAccel(0x32, 0x10);  //Write 10h into INT1_THS;          // Threshold (THS) = 16LSBs * 15.625mg/LSB = 250mg.
  sendToAccel(0x33, 0x00);  //Write 00h into INT1_DURATION;     // Duration = 1LSBs * (1/10Hz) = 0.1s.
  //readRegister();  //Dummy read to force the HP filter to set reference acceleration/tilt value
  sendToAccel(0x30, 0x2A);  //Write 2Ah into INT1_CFG;          // Enable XLIE, YLIE, ZLIE interrupt generation, OR logic.
}

void disableHpf() {
  sendToAccel(0x21, 0x00);
}

/*
void configInterrupts() {

  // Configure CTRL_REG1: Enable X, Y, Z axes, ODR = 100Hz
  sendToAccel(LIS3DH_CTRL_REG1, lis3dh_reg_t{ .ctrl_reg1 = {
    .xen = 1,
    .yen = 1,
    .zen = 1,
    .lpen = 0,
    .odr = LIS3DH_ODR_100Hz
  }});

  // Configure CTRL_REG2: High-pass filter enabled for IA1
  sendToAccel(LIS3DH_CTRL_REG2, lis3dh_reg_t{ .ctrl_reg2 = {
    .hp = 0x1, // HP_IA1 enabled
    .fds = 1,  // Filtered data enabled
    .hpcf = LIS3DH_AGGRESSIVE,
    .hpm = LIS3DH_NORMAL_WITH_RST
  }});

  // Configure CTRL_REG3: Route IA1 interrupt to INT1
  sendToAccel(LIS3DH_CTRL_REG3, lis3dh_reg_t{ .ctrl_reg3 = {
    .i1_ia1 = 1
  }});

  // Configure CTRL_REG4: Full scale ±2g, High-resolution disabled
  sendToAccel(LIS3DH_CTRL_REG4, lis3dh_reg_t{ .ctrl_reg4 = {
    .fs = LIS3DH_2g,
    .bdu = 0
  }});

  // Configure CTRL_REG5: No latching interrupts
  sendToAccel(LIS3DH_CTRL_REG5, lis3dh_reg_t{ .ctrl_reg5 = {
    .lir_int1 = 0
  }});

  // Configure CTRL_REG6: Interrupt polarity (active low)
  sendToAccel(LIS3DH_CTRL_REG6, lis3dh_reg_t{ .ctrl_reg6 = {
    .int_polarity = 1
  }});

  // Configure INT1_THS: Threshold = 250mg (16 * 15.625mg)
  sendToAccel(LIS3DH_INT1_THS, lis3dh_reg_t{ .int1_ths = {
    .ths = 16
  }});

  // Configure INT1_DURATION: Duration = 0.1s
  sendToAccel(LIS3DH_INT1_DURATION, lis3dh_reg_t{ .int1_duration = {
    .d = 0
  }});

  // Configure INT1_CFG: Enable XHIE, YHIE, ZHIE with OR logic
  sendToAccel(LIS3DH_INT1_CFG, lis3dh_reg_t{ .int1_cfg = {
    .xhie = 1,
    .yhie = 1,
    .zhie = 1,
    .aoi = 0
  }});

  inline void sendToAccel(uint8_t addr, lis3dh_reg_t val) {
    sendToAccel(addr, val.byte)
  }
  */

void sendToAccel(uint8_t addr, uint8_t val) {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(byte(addr));  // register address
  Wire.write(val);         // register value
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
  Wire.requestFrom(ACCEL_ADDRESS, 1);
  while (Wire.available())  // slave may send less than requested
  {
    *output = Wire.read();  // receive a byte as a proper uint8_t
  }
}

void goToSleep() {
  stop_leds();
  configInterrupts();  // only really needs to be called once
  // TODO: set the accel to a lower power mode
  attachInterrupt(0, wakeUp, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
  disableHpf();
  led_init();
}

// ---
// interrupt/timer stuff

ISR(TIMER2_COMPA_vect) {
  g_bitpos++;
  g_bitpos &= 7;

  // Update all ports with their current timeslice, preserving unused pins
  PORTB = (PORTB & ~PORT_B_MASK) | (g_timeslice_b[g_bitpos] & PORT_B_MASK);
  PORTC = (PORTC & ~PORT_C_MASK) | (g_timeslice_c[g_bitpos] & PORT_C_MASK);
  PORTD = (PORTD & ~PORT_D_MASK) | (g_timeslice_d[g_bitpos] & PORT_D_MASK);

  TCNT2 = 0;  // Reset Timer2 counter for next time slice

  // Double OCR2A for the next time slice to implement Binary Code Modulation.
  // Time slice durations will be proportional to 2^0, 2^1, 2^2, ..., 2^7, then repeat.
  OCR2A <<= 1;
  if (g_bitpos == 0) {
    OCR2A = 1;  // Reset OCR2A to 1 at the start of each 8-bit BCM cycle
  }

  if (g_bitpos == 7) {
    g_tick = 1;  // Signal the end of an 8-bit BCM cycle (purpose of g_tick needs clarification)
  }
}

void wakeUp() {
  // Just a handler for the pin interrupt.
  // use this if the accel config needs the interrupt to be cleared:
  // clearInterrupt();
}gg