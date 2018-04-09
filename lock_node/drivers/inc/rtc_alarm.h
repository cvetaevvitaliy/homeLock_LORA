#ifndef RTC_ALARM_H
#define RTC_ALARM_H

#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define INIT_YEAR			0x18
#define INTI_MONTH		0x04
#define INIT_DAY			0x08
#define INIT_HOUR			0x15
#define INIT_MINITUE	0x32
#define INIT_SECOND		0x00

#define INIT_ALARM_HOUR			INIT_HOUR
#define INIT_ALARM_MINUTE  (INIT_MINITUE + 3)

extern RTC_HandleTypeDef hrtc;

void init_RTC(void);
void setAlarm(uint8_t hour, uint8_t minute);
void calDateTime(uint8_t* calTime);

#endif 

// ----   end		------
