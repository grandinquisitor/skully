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

void showError(uint8_t);

uint16_t getAngle(bool reset);

bool getClick();

void configInterrupts();

void disableHpf();

void sendToAccel(uint8_t addr, uint8_t val);

void clearInterrupt();

void goToSleep();

void wakeUp();

#endif // __MAIN_H__