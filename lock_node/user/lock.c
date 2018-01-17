#include "lock.h"
#include "usart.h"
#include "string.h"
#include "delay.h"
#include "globalVar.h"

#define LOCK_MAX_SIZE 60
LOCK_COM_STRUCT lock_com;

void usart2_rx_enbale_int(void){	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStructure.Pin   = GPIO_PIN_3;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
	
  HAL_NVIC_SetPriority((IRQn_Type)EXTI2_3_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_3_IRQn);
}

void usart2_tx_gpio_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	 GPIO_InitStruct.Pin       = USART2_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
	#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	#endif
 
  HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);
}

void send_cmd_to_lock( uint8_t cmd, uint8_t* data, uint8_t data_len ){
	TMsg Msg;
	uint16_t temp_sum = 0;

	usart2_tx_gpio_init();
	HAL_GPIO_WritePin( USART2_TX_GPIO_PORT, USART2_TX_PIN, GPIO_PIN_RESET );
	Delay_nms(10);
	HAL_GPIO_WritePin( USART2_TX_GPIO_PORT, USART2_TX_PIN, GPIO_PIN_SET );
	Delay_nms(100);
	
	usart2_tx_AFGPIO_init();
	lock_com.cmd_type = cmd;
	lock_com.wait_time = local_ticktime();
	lock_com.rx_state = RX_BUSY;
	Msg.Data[0] = LOCK_HEADER;
	Msg.Data[3] = data_len + 1;
	Msg.Data[4] = cmd;
	temp_sum = Msg.Data[3];
	temp_sum += Msg.Data[4];
	for(uint8_t i = 0; i < data_len; i++){
		Msg.Data[i+5] = data[i];
		temp_sum += Msg.Data[i+5];
	}
	Msg.Data[1] = (uint8_t)(temp_sum >> 8);
	Msg.Data[2] = (uint8_t)temp_sum;
	Msg.Len = data_len + 5;
	
	UART_SendMsg(&Msg, USART2);
}


