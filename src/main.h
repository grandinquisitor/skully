#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdint.h>
#include <stdbool.h>

#include "lis3dh_reg.h"

void led_init(void);

void stop_leds(void);

void setup_timer();

void start_timer();

void stop_timer();

void other_hardware_config();

void led_encode_timeslices(uint8_t intensity[]);

bool connect_accel();

void test_delay();

void showError(uint8_t);

uint16_t getAngle(bool reset);

bool getClick();

void configInterrupts();

void disableHpf();

void sendToAccel(uint8_t addr, uint8_t val);

void clearInterrupt();

void goToSleep();

void wakeUp();

bool setupAccel(lis3dh_odr_t, bool);

void readFromAccel(uint8_t addr, uint8_t* output);

uint8_t readRegionFromAccel(uint8_t addr, uint8_t* output, uint8_t length);

uint8_t readByteFromAccel(uint8_t addr);

void setupInterrupt();

bool setupAccelStartup();

bool setupAccelSleep();

bool setupAccelReawaken();

#endif // __MAIN_H__