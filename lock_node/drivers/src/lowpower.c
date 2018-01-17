#include "lowpower.h"
#include "stm32l0xx_hal.h"
//PA1  -- LORA POWER CONTROL, High valid
//PB12 -- Finger VCC control, High valid

void init_PowerControl_Configuration(void){
	GPIO_InitTypeDef	GPIO_InitStructure;

	GPIO_InitStructure.Pin 	 = GPIO_PIN_1;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
	
	GPIO_InitStructure.Pin 	 = GPIO_PIN_12;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void powerOn_LoRa(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); }

void powerOff_LoRa(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
}

void powerOn_finger(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); }

void powerOff_finger(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
}

void gpio_reInit_before_stop(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Pin 	 = GPIO_PIN_All;
	GPIO_InitStructure.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	__HAL_RCC_GPIOA_CLK_DISABLE();
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	__HAL_RCC_GPIOB_CLK_DISABLE();
	
		HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	__HAL_RCC_GPIOC_CLK_DISABLE();	
}

void enter_StopMode(void){
	
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

