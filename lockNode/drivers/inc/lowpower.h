#ifndef _LOWPOWER_H_
#define _LOWPOWER_H_
#include "stm32l0xx.h"

enum POWER_STATE {
	POWERON, 
	POWEROFF
};

/*
 @brief POWER±àºÅ
 */
enum POWER_NO {
	POWER_NO_0 = 0,
	POWER_NO_1 = 1
};
struct PowerDefine {
	enum POWER_NO power_no;
	GPIO_TypeDef* pin_group;
	uint16_t pin;
	enum POWER_STATE status;
};

extern struct PowerDefine power[2];

void init_PowerControl_Configuration(void);
void powerOn_LoRa(void);
void powerOff_LoRa(void);
void powerOn_finger(void);
void powerOff_finger(void);

void gpio_reInit_before_stop(void);
void enter_StopMode(void);

#endif
