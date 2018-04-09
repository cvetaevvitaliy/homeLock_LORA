#ifndef __USART_H__
#define __USART_H__

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_hal_def.h"

#define TMsg_MaxLen  150

/* Exported defines ----------------------------------------------------------*/
#define UART1_RxBufferSize 900  //12+139*3=429 (128+2+9)*12 = 1668
#define UART1_TxBufferSize 150   // 128+2+9 = 139

#define UART2_TxBufferSize 60
#define UART2_RxBufferSize 60

/* User can use this section to tailor USARTx/UARTx instance used and associated resources */
/* Definition for USARTx clock resources */

#define USART2_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define DMA1_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define USART2_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART1_CLK_ENABLE()              __USART1_CLK_ENABLE();
#define USART1_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USART2_FORCE_RESET()             __USART2_FORCE_RESET()
#define USART2_RELEASE_RESET()           __USART2_RELEASE_RESET()
#define USART1_FORCE_RESET()             __USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_RX_PIN                    GPIO_PIN_3
#define USART2_RX_GPIO_PORT              GPIOA

#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_RX_PIN                    GPIO_PIN_10
#define USART1_RX_GPIO_PORT              GPIOA

#ifdef USE_STM32L0XX_NUCLEO
#define USART2_TX_AF                     GPIO_AF4_USART2
#define USART2_RX_AF                     GPIO_AF4_USART2
#define USART1_TX_AF                     GPIO_AF4_USART1
#define USART1_RX_AF                     GPIO_AF4_USART1
#endif

typedef struct{
  uint8_t Len;
  uint8_t Data[TMsg_MaxLen];
} TMsg;

typedef struct{
	uint32_t rx_timeTick;
	uint8_t rx_status;
}Rx_para;
/* exported variables --------------------------------------------------------*/
extern  uint8_t UART2_RxBuffer[UART2_RxBufferSize];
extern  uint8_t UART1_RxBuffer[UART1_RxBufferSize];
extern UART_HandleTypeDef UartHandle1, UartHandle2;
extern Rx_para usart1_rx_flag, usart2_rx_flag;
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void usart2_tx_AFGPIO_init(void);
void USARTConfig(void);
int UART_ReceivedMSG(TMsg *Msg);
void UART_SendMsg(TMsg *Msg, USART_TypeDef* Usartx);
void USART_DMA_Configuration(void);

#endif
