#ifndef __LED_H__
#define __LED_H__

#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

/*
 @brief LED的状态
 */
enum LED_STATE {
	LEDON, 
	LEDOFF
};

/*
 @brief LED编号
 */
enum LED_NO {
	LED_NO_0 = 0,
	LED_NO_1 = 1
};

/*
 @brief LED的最大数量
 */
#define LED_MAX_SIZE (2)

/*
 @brief LED定义
 */
struct LedDefine {
	enum LED_NO led_no;
	// PIN_GROUP（GPIOA ...）
	GPIO_TypeDef* pin_group;
	// PIN（GPIO_Pin_1 ...）
	uint16_t pin;
	// LEDON | LEDOFF
	enum LED_STATE defaultstatus;
};

void init_led(void);

/*
 @brief 点亮LED
 @param led_no LED编号
 */
uint8_t LED_ON(enum LED_NO led_no);

/*
 @brief 熄灭LED
 @param led_no LED编号
 */
uint8_t LED_OFF(enum LED_NO led_no);

uint8_t LED_TOGGLE(enum LED_NO led_no);
#endif
