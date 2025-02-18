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

void show_error(uint8_t);

uint16_t get_angle(bool reset);

bool get_click();

void configInterrupts();

void disableHpf();

void send_to_accel(uint8_t addr, uint8_t val);

void clear_int1_interrupt();

void go_to_sleep();

bool setup_accel(lis3dh_odr_t, bool);

void read_from_accel(uint8_t addr, uint8_t* output);

uint8_t read_from_accel(uint8_t addr, uint8_t* output, uint8_t length);

uint8_t read_from_accel(uint8_t addr);

void setup_interrupt();

bool setup_accel_startup();

bool setup_accel_sleep();

bool setup_accel_reawake();

void send_to_accel(uint8_t addr, lis3dh_reg_t val);

bool send_to_accel_and_verify(uint8_t addr, lis3dh_reg_t val);

#endif // __MAIN_H__
