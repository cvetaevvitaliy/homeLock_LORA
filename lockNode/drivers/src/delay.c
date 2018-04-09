#include "stm32l0xx.h"
#include "Delay.h"
//todo  system freq
/*
@16MHz
*/
void Delay_1us(void){
	uint16_t i;
	for(i = 2; i > 0; i--)        
	{
	}
}

void Delay_nus(uint16_t n){
	uint16_t i;
	for(i = n; i > 0;i--)
	{
		Delay_1us();
	}
}

void Delay_1ms(void){
	uint16_t i;
	for(i = 0x186A; i > 0; i--)    
	{
	}
}

void Delay_nms(uint16_t n){
	uint16_t i;
	for(i = n; i > 0; i--)
	{
		Delay_1ms();
	}
}
