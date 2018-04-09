#include "lock.h"
#include "usart.h"
#include "string.h"
#include "delay.h"
#include "globalVar.h"
#include "schd.h"

#define LOCK_MAX_SIZE 			60

LOCK_COM_STRUCT lock_com;
volatile LOCKUNIT lockUnit = {0x0001, 0};

void init_lock_com(void){
	lock_com.cmd_type = LOCK_CMD_IDLE;
	lock_com.retry_times = 0;
	memset(lock_com.last_data, 0, lock_com.last_data_len);
	lock_com.last_data_len = 0;
}

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

static void lock_CMD_resend_setting(uint8_t cmd, uint8_t*data, uint8_t dataLen){
	lock_com.cmd_type = cmd;
	lock_com.last_data_len =  dataLen;
	memcpy(lock_com.last_data, data, lock_com.last_data_len);
	lock_com.retry_times = 0;
	lock_com.timeoutTick = local_ticktime();
}

void send_cmd_to_lock( uint8_t cmd, uint8_t* data, uint8_t data_len ){
	TMsg Msg;
	uint16_t temp_sum = 0;
	
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
	
	usart2_tx_gpio_init();
	HAL_GPIO_WritePin( USART2_TX_GPIO_PORT, USART2_TX_PIN, GPIO_PIN_RESET );
	Delay_nms(10);
	HAL_GPIO_WritePin( USART2_TX_GPIO_PORT, USART2_TX_PIN, GPIO_PIN_SET );
	Delay_nms(100);
	
	usart2_tx_AFGPIO_init();
	
	UART_SendMsg(&Msg, USART2);	
	lock_CMD_resend_setting( cmd, data, data_len);
}

void lock_init(void){
	uint8_t temp_data[20] = {0x00};
	
	temp_data[0] = 0x02;
	send_cmd_to_lock(LOCK_CMD_INIT, temp_data, 1 );				//≥ı ºªØ÷∏¡Ó
	while(lock_com.cmd_type != LOCK_CMD_IDLE);
	temp_data[0] = 0x00;
	temp_data[3] = 0x01;
	temp_data[7] = 0x01;
	send_cmd_to_lock(LOCK_CMD_REGISTER, temp_data, 8 );  //enble door bell 	
}

