#ifndef _LOWPOWER_H_
#define _LOWPOWER_H_
#include "stm32l0xx.h"

void init_PowerControl_Configuration(void);
void powerOn_LoRa(void);
void powerOff_LoRa(void);
void powerOn_finger(void);
void powerOff_finger(void);

void gpio_reInit_before_stop(void);
void enter_StopMode(void);

#endif
