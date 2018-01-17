#include "usart.h"

/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle2;//lock
UART_HandleTypeDef UartHandle1;//finger

uint8_t UART2_RxBuffer[UART2_RxBufferSize];
uint8_t UART2_TxBuffer[UART2_TxBufferSize];
uint8_t UART1_RxBuffer[UART1_RxBufferSize];
uint8_t UART1_TxBuffer[UART1_TxBufferSize];
Rx_para usart1_rx_flag, usart2_rx_flag;
/**
  * @brief  Send a message via UART
  * @param  Msg the pointer to the message to be sent
  * @retval None
  */
void UART_SendMsg(TMsg *Msg, USART_TypeDef* Usartx){
	if(Usartx == USART1)
		HAL_UART_Transmit(&UartHandle1, (uint8_t*)&Msg->Data[0], Msg->Len, 500);
	else if(Usartx == USART2)
			HAL_UART_Transmit(&UartHandle2, (uint8_t*)&Msg->Data[0], Msg->Len, 500);
}
void usart2_tx_AFGPIO_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin       = USART2_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
#endif
  
  GPIO_InitStruct.Alternate = USART2_TX_AF;
  HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);
}
/**
 * @brief  Configure the USART
 * @retval None
 */
void USARTConfig(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USART2_TX_GPIO_CLK_ENABLE();
  USART2_RX_GPIO_CLK_ENABLE();
	USART1_TX_GPIO_CLK_ENABLE();
  USART1_RX_GPIO_CLK_ENABLE();
  /* Enable USART2 clock */
  USART2_CLK_ENABLE();
	USART1_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
	usart2_tx_AFGPIO_init();
  
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
#endif
	GPIO_InitStruct.Pin       = USART1_TX_PIN;
	GPIO_InitStruct.Alternate = USART1_TX_AF;
	HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin 			= USART2_RX_PIN;
  GPIO_InitStruct.Alternate = USART2_RX_AF;	
  HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin			  = USART1_RX_PIN;
  GPIO_InitStruct.Alternate = USART1_RX_AF;
	HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);
  
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  UartHandle2.Instance        = USART2;
  UartHandle2.Init.BaudRate   = 9600;
  UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle2.Init.StopBits   = UART_STOPBITS_1;
  UartHandle2.Init.Parity     = UART_PARITY_NONE;
  UartHandle2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle2.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle2) != HAL_OK){
//  Error_Handler();
    while(1);
  }

  
	UartHandle1.Instance        = USART1;
  UartHandle1.Init.BaudRate   = 57600;
  UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle1.Init.StopBits   = UART_STOPBITS_1;
  UartHandle1.Init.Parity     = UART_PARITY_NONE;
  UartHandle1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle1.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle1) != HAL_OK){
//  Error_Handler();
    while(1);
  }
 	
	HAL_UART_Receive_IT(&UartHandle2, (uint8_t*)UART2_RxBuffer, UART2_RxBufferSize);	
//	__HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_WUF);//wakeup from stop mode interrupt
	HAL_NVIC_SetPriority((IRQn_Type)USART2_IRQn, 2, 0); 
	HAL_NVIC_EnableIRQ((IRQn_Type)USART2_IRQn);
	
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t*)UART1_RxBuffer, UART1_RxBufferSize);	
	HAL_NVIC_SetPriority((IRQn_Type)USART1_IRQn, 2, 1); 
	HAL_NVIC_EnableIRQ((IRQn_Type)USART1_IRQn);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	huart->ErrorCode = HAL_UART_ERROR_NONE;
	if(huart->Instance == USART1)
		HAL_UART_Receive_IT(&UartHandle1, (uint8_t*)UART1_RxBuffer, UART1_RxBufferSize);
	if(huart->Instance == USART2)
		HAL_UART_Receive_IT(&UartHandle2, (uint8_t*)UART2_RxBuffer, UART2_RxBufferSize);
}
void USART1_IRQHandler(void){
	HAL_UART_IRQHandler(&UartHandle1);
//	__HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);
}

void USART2_IRQHandler(void){
	HAL_UART_IRQHandler(&UartHandle2);
	
//	__HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_RXNE);
}
