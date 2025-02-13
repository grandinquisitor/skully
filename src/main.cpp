#include "main.h"

#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <LowPower.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>

#include "animation.h"
#include "fxpt_atan2.h"
#include "lis3dh_reg.h"
#include "util.h"

#define TIMER_PRESCALER 64
#define FRAME_INTERVAL_MS 128

// Sum of all BCM time slice durations (each slice is OCR2A+1 ticks)
// 2 + 3 + 5 + 9 + 17 + 33 + 65 + 129 = 263
#define BCM_CYCLE_TICKS_SUM 263
#define BCM_CYCLES_PER_ANIMATION \
  ((F_CPU / 1000 * FRAME_INTERVAL_MS) / (BCM_CYCLE_TICKS_SUM * TIMER_PRESCALER))
#if BCM_CYCLES_PER_ANIMATION == 0
#error "update rate is < 1"
#elif BCM_CYCLES_PER_ANIMATION >= 255
#error "update rate is > 255, won't fit in uint8_t"
#endif

#define TIMER_USED 2

#define I2C_SPEED_400KHZ 400000
#define I2C_SPEED_100KHZ 100000
#define I2C_SPEED_50KHZ 50000
#define I2C_SPEED_25KHZ 25000
#define I2C_SPEED_10KHZ 10000

#if F_CPU == 1000000UL
#define I2C_SPEED I2C_SPEED_10KHZ
#elif F_CPU == 2000000UL
#define I2C_SPEED I2C_SPEED_100KHZ
#endif

// goal: see what changes reduce the jittering
// it does

constexpr uint16_t GET_ANGLE_FREQ = 8000;
constexpr uint16_t GET_CLICK_FREQ = 2000;

// TODO:
// try in platformIO
// why is the framerate so slow with the accelerometer turned on?
// does millis() not work? (i.e. countdown not working)
// doesn't that run in timer0?

enum class BLEND_LEVEL { BLEND_0, BLEND_50, BLEND_75, BLEND_87_5 };

typedef enum { PORTB_ENUM = 0, PORT_C_ENUM = 1, PORT_D_ENUM = 2 } PortIndex;

typedef struct {
  PortIndex port;
  uint8_t pin;
} LedMapping;

constexpr uint8_t NUM_LEDS = 18;
constexpr LedMapping LED_MAP[NUM_LEDS] = {
    {PORT_D_ENUM, PORTD0},  // PORTD0
    {PORT_C_ENUM, PORTC1},  // PORTC1
    {PORT_C_ENUM, PORTC0},  // PORTC0
    {PORTB_ENUM, PORTB5},   // PORTB5
    {PORTB_ENUM, PORTB4},   // PORTB4
    {PORTB_ENUM, PORTB3},   // PORTB3
    {PORTB_ENUM, PORTB2},   // PORTB2
    {PORTB_ENUM, PORTB1},   // PORTB1
    {PORTB_ENUM, PORTB0},   // PORTB0
    {PORT_D_ENUM, PORTD7},  // PORTD7
    {PORT_D_ENUM, PORTD6},  // PORTD6
    {PORT_D_ENUM, PORTD5},  // PORTD5
    {PORTB_ENUM, PORTB7},   // PORTB7
    {PORTB_ENUM, PORTB6},   // PORTB6
    {PORT_D_ENUM, PORTD4},  // PORTD4
    {PORT_D_ENUM, PORTD3},  // PORTD3
    {PORT_C_ENUM, PORTC3},  // PORTC3
    {PORT_C_ENUM, PORTC2}   // PORTC2
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

constexpr uint8_t PORT_B_MASK = _make_bitmap(PORTB_ENUM);
constexpr uint8_t PORT_C_MASK = _make_bitmap(PORT_C_ENUM);
constexpr uint8_t PORT_D_MASK = _make_bitmap(PORT_D_ENUM);

// Global variables
uint8_t g_timeslice[3][8] = {0};
uint8_t *g_timeslice_b = g_timeslice[PORTB_ENUM];
uint8_t *g_timeslice_c = g_timeslice[PORT_C_ENUM];
uint8_t *g_timeslice_d = g_timeslice[PORT_D_ENUM];

#define IS_CONTIGUOUS(a, b, c) \
  (((a) + (b) + (c) == 3) && ((a) * (b) * (c) == 0))
static_assert(IS_CONTIGUOUS(PORTB_ENUM, PORT_C_ENUM, PORT_D_ENUM),
              "weird values");

// volatile uint8_t g_timeslice_b[8];
// volatile uint8_t g_timeslice_c[8];
// volatile uint8_t g_timeslice_d[8];

volatile uint8_t g_tick = 0;
volatile uint8_t g_bitpos = 0;
uint8_t brightness[NUM_LEDS] = {0};

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

void setup() {

#if F_CPU == 1000000UL
// assume hardware presaler is set
#elif F_CPU == 2000000UL
  clock_prescale_set(clock_div_4);
#elif F_CPU == 4000000UL
  clock_prescale_set(clock_div_2);
#elif F_CPU == 8000000UL
// no prescaler needed
#endif

  led_init();

  // Initialize all brightness values to 0
  // memset()
  // for (uint8_t i = 0; i < NUM_LEDS; i++) {
  //   brightness[i] = 0;
  // }

  led_encode_timeslices(brightness);

  other_hardware_config();

  sei();

  if (!NO_LIS) {
    Wire.begin();

#ifdef I2C_SPEED
#if ((F_CPU / 18) < I2C_SPEED)
#error "I2C speed too high"
#else
    Wire.setClock(I2C_SPEED);
#endif
#endif

    if (!connect_accel()) {
      showError(5);
    }
  }

  start_timer();
}

// TODO:
// move setup timer out of led setup
// fix timing: try vs. without accel
// does it pull every time you call getAngle()?

// problem:
// 

void loop() {
  static uint8_t g_frame_counter = 0;
  static bool animation_reset = true;
  static bool accel_reset = true;

  while (g_tick == 0) { /*wait for g_tick to be non-zero*/
  }
  g_tick = 0;  // consume the tick

  // Update animation based on calculated interval
  // problem: this should run once per ms in order to match the old system
  if (++g_frame_counter >= BCM_CYCLES_PER_ANIMATION) {
    g_frame_counter = 0;
    bool animation_off = calculate_next_frame(brightness, getAngle(accel_reset),
                                          getClick(), animation_reset);
    animation_reset = false;
    accel_reset = false;

    // test click is not latched on (works as expected)
    // if (getClick()) {
    //   brightness[0] = 64;
    // } else {
    //   brightness[0] >>= 1;
    // }

    if (animation_off && !NO_LIS) {
      goToSleep();
      animation_reset = true;
    }
  }

  led_encode_timeslices(brightness);
}

void resetI2CBus() {
  // Configure SCL as output and manually pulse it
  DDRC |= _BV(PC5);    // SCL on PC5 (Arduino A5) - adjust for your board
  PORTC &= ~_BV(PC5);  // Ensure SCL starts low

  for (int i = 0; i < 9; i++) {
    PORTC |= _BV(PC5);  // SCL high
    _delay_us(10);
    PORTC &= ~_BV(PC5);  // SCL low
    _delay_us(10);
  }

  // Restore SCL to input (with pull-up)
  DDRC &= ~_BV(PC5);
  PORTC |= _BV(PC5);
}

void configureI2CFor1MHz() {
  // Set TWI prescaler to 4 (TWSR |= _BV(TWPS0))
  TWSR |= _BV(TWPS0);  // Prescaler = 4
  TWBR = 1;            // TWBR = ((F_CPU / SCL_Freq) - 16) / (2 * Prescaler)
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
}

void stop_leds(void) {
  // turn all leds off, set to INPUT_PULLUP
  PORTB |= PORT_B_MASK;
  PORTC |= PORT_C_MASK;
  PORTD |= PORT_D_MASK;

  DDRB &= ~PORT_B_MASK;
  DDRC &= ~PORT_C_MASK;
  DDRD &= ~PORT_D_MASK;
}

void start_timer() {
  bool interruptWasEnabled = SREG & _BV(SREG_I);
  if (interruptWasEnabled) {
    cli();
  }

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

  if (interruptWasEnabled) {
    sei();
  }
}

void stop_timer() {
  // once the timer is set up, we could just call cli() and sei() instead of
  // individually just stopping/starting this timer

  bool interruptWasEnabled = SREG & _BV(SREG_I);
  if (interruptWasEnabled) {
    cli();
  }

  // stops the ISR
  TIMSK2 &= ~_BV(OCIE2A);

  // stops the timer clock -- probably not strictly necessary
  // TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));

  // reset the timer/counter state (not strictly needed)
  // TCNT2 = 0;  // Reset the timer counter
  // OCR2A = 0;  // Reset the output compare register

  if (interruptWasEnabled) {
    sei();
  }
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

void led_encode_timeslices(uint8_t intensity[]) {
  // Clear all timeslices (start with all LEDs off)

  // memset(g_timeslice_b, 0xFF, 8);
  // for (uint8_t bitpos = 0; bitpos < 8; bitpos++) {
  //   g_timeslice_b[bitpos] = 0xFF;
  //   g_timeslice_c[bitpos] = 0xFF;
  //   g_timeslice_d[bitpos] = 0xFF;
  // }
  memset(g_timeslice, 0xFF, sizeof(g_timeslice));

  // For each LED
  for (uint8_t led = 0; led < NUM_LEDS; led++) {
    // For each bit position in the brightness value
    for (uint8_t bitpos = 0, bitmask = 1; bitpos < 8; bitpos++, bitmask <<= 1) {
      if (intensity[led] & bitmask) {
        // If this bit is set, turn on the LED for this timeslice
        // TODO: LUT this bit shift
        uint8_t pin_mask = ~_BV(LED_MAP[led].pin);  // Inverted because LOW = ON

        g_timeslice[LED_MAP[led].port][bitpos] &= pin_mask;
        // switch (LED_MAP[led].port) {
        //   case PORT_B:
        //     g_timeslice_b[bitpos] &= pin_mask;
        //     break;
        //   case PORT_C:
        //     g_timeslice_c[bitpos] &= pin_mask;
        //     break;
        //   case PORT_D:
        //     g_timeslice_d[bitpos] &= pin_mask;
        //     break;
        // }
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
    // TODO: overwritten by configInterrupts anyway, rewrite using a consistent
    // driver
    lis.setRange(LIS3DH_RANGE_2_G);  // 2, 4, 8 or 16 G!
    lis.setClick(2, CLICKTHRESHHOLD);
    disableHpf();
    return true;
  }
}

// used to test timing
void test_delay() {
  DDRD |= _BV(DDD0);
  while (1) {
    for (uint8_t i = 0; i < 5; i++) {
      PORTD &= ~_BV(PORTD0);
      delay(1000);
      PORTD |= _BV(PORTD0);
      delay(1000);
    }
    for (uint8_t i = 0; i < 5; i++) {
      PORTD &= ~_BV(PORTD0);
      _delay_ms(1000);
      PORTD |= _BV(PORTD0);
      _delay_ms(1000);
    }
  }
}

void showError(uint8_t times) {
  uint8_t old_ddrd4_bit = DDRD & _BV(DDD4);
  DDRD |= _BV(DDD4);

  for (uint8_t i = 0; i < 5; i++) {
    PORTD &= ~_BV(PORTD4);
    delay(500);
    PORTD |= _BV(PORTD4);
    delay(500);
  }

  if (old_ddrd4_bit) {
    DDRD |= _BV(DDD4);  // If original bit was 1, set it back to 1
  } else {
    DDRD &= ~_BV(DDD4);  // If original bit was 0, set it back to 0
  }
}

uint16_t getAngle(bool reset) {
  if (NO_LIS) {
    return 0;
  }

  constexpr uint16_t ACCEL_ANGLE_OFFSET =
      0x3fff;  // based on the orientation of the chip on the board
  static uint32_t accelCounter = 0;
  static uint16_t roll = 0;
  static int32_t xFilter, yFilter;

  bool got_new_read = false;
  if (reset) {
    lis.read();
    accelCounter = millis();  // not strictly necessary
    xFilter = setFilterValue(lis.x);
    yFilter = setFilterValue(lis.y);
    got_new_read = true;
  } else {
    if (countDown(&accelCounter, GET_ANGLE_FREQ)) {
      lis.read();
      smoothInt(lis.x, 1, &xFilter);
      smoothInt(lis.y, 1, &yFilter);
      got_new_read = true;
    }
  }

  if (got_new_read) {
    roll = fxpt_atan2(getFilterValue(xFilter), getFilterValue(yFilter)) +
           ACCEL_ANGLE_OFFSET;
  }

  return roll;
}

bool getClick() {
  if (NO_LIS) {
    return true;
  }

  static uint32_t accelCounter = 0;

  if (countDown(&accelCounter, GET_CLICK_FREQ)) {
    uint8_t click = lis.getClick();
    if (!(click == 0 || !(click & 0x30))) {
      // not necessary to clear, based on the configuration
      // clearInterrupt();
      return true;
    }
  }
  return false;
}

// TODO: replace with proper constants and use lis3dh_reg_t for readability
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
  sendToAccel(0x30, 0x2A);  // Write 2Ah into INT1_CFG;          // Enable XHIE,
                            // YHIE, ZHIE interrupt generation, OR logic.
}

void disableHpf() { sendToAccel(0x21, 0x00); }

void sendToAccel(uint8_t addr, uint8_t val) {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(byte(addr));  // register address
  Wire.write(val);         // register value
  Wire.endTransmission();  // stop transmitting
}

void readFromAccel(uint8_t addr, uint8_t *output) {
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

void clearInterrupt() {
  uint8_t dataRead;
  readFromAccel(LIS3DH_INT1_SRC, &dataRead);
}

void goToSleep() {
  stop_timer();
  stop_leds();
  configInterrupts();  // only really needs to be called once
  // TODO: set the accel to a lower power mode
  attachInterrupt(0, wakeUp, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
  disableHpf();
  led_init();
  start_timer();
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
  // Time slice durations will be proportional to 2^0, 2^1, 2^2, ..., 2^7, then
  // repeat.
  OCR2A <<= 1;
  if (g_bitpos == 0) {
    OCR2A = 1;  // Reset OCR2A to 1 at the start of each 8-bit BCM cycle
  }

  if (g_bitpos == 7) {
    g_tick = 1;  // Signal the end of an 8-bit BCM cycle (purpose of g_tick
                 // needs clarification)
  }
}

void wakeUp() {
  // Just a handler for the pin interrupt.
  // use this if the accel config needs the interrupt to be cleared:
  // clearInterrupt();
}