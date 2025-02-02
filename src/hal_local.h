#ifndef __HAL_LOCAL_H__
#define __HAL_LOCAL_H__

#include <stdbool.h>
#include <stdint.h>

bool initAccel();
void init_mcu();
void enable_output();
void disable_output();
void wakeUp();
void goToSleep();
void configInterrupts();
void sendToAccel(uint8_t addr, uint8_t val);
void clearInterrupt();
void readFromAccel(uint8_t addr, uint8_t* output);
void disableHpf();
void update_accel();
int16_t getX();
int16_t getY();
bool got_click();


#endif // __HAL_LOCAL_H__

