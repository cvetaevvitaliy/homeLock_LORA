#include "GlobalVar.h"
#include "schd.h"
#include "stm32l0xx_hal.h"

/*
 SysTick更新时间，用于计算时间间隔（每10ms增加一次）
 */
__IO uint32_t _localTimeTick_ms = 0;
#define SYSTEMTICK_PERIOD_MS  10

void SysTick_Handler(void) {
	_localTimeTick_ms += SYSTEMTICK_PERIOD_MS;
	HAL_IncTick();	
	// 调用定时触发器
	Schd_Run(_localTimeTick_ms);
}

uint32_t local_ticktime() {
	return _localTimeTick_ms;
}

uint8_t timeout(uint32_t last_time, uint32_t ms) {
	return ((_localTimeTick_ms - last_time) > ms);
}

/*
 @brief 延时
 */
void delay_us(uint32_t nus) {
	unsigned long delay_time = nus * 12;
	while (delay_time--)
		;
}

/*
 @brief 喂狗
 */
void watchdog(void) {

	//Feed Dog
	IWDG->KR = 0XAAAA;
}
