#include "spi.h"

uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
SPI_HandleTypeDef hnucleo_Spi;

/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
	
static void SPIx_MspInit(void){
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
  NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();
  
  /* Configure SPI SCK */
  gpioinitstruct.Pin 				= NUCLEO_SPIx_SCK_PIN;
  gpioinitstruct.Mode 			= GPIO_MODE_AF_PP;
  gpioinitstruct.Pull  			= GPIO_PULLUP;
  gpioinitstruct.Speed 			= GPIO_SPEED_FREQ_VERY_HIGH;
  gpioinitstruct.Alternate  = NUCLEO_SPIx_SCK_AF;
  HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &gpioinitstruct);

  /* Configure SPI MISO and MOSI */ 
  gpioinitstruct.Pin 				= NUCLEO_SPIx_MOSI_PIN;
  gpioinitstruct.Alternate  = NUCLEO_SPIx_MISO_MOSI_AF;
  gpioinitstruct.Pull  			= GPIO_PULLDOWN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);
  
  gpioinitstruct.Pin 				= NUCLEO_SPIx_MISO_PIN;
  HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/ 
  /* Enable SPI clock */
  NUCLEO_SPIx_CLK_ENABLE();
}

/**
  * @brief  Initialize SPI HAL.
  * @retval None
  */
void SPIx_Init(void){
  if(HAL_SPI_GetState(&hnucleo_Spi) == HAL_SPI_STATE_RESET){
    /* SPI Config */
    hnucleo_Spi.Instance = NUCLEO_SPIx;
      /* SPI baudrate is set to 8 MHz maximum (PCLK2/SPI_BaudRatePrescaler = 32/4 = 8 MHz) 
       to verify these constraints:
          - PCLK2 max frequency is 32 MHz 
       */ 
    hnucleo_Spi.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_4;
    hnucleo_Spi.Init.Direction 					= SPI_DIRECTION_2LINES;
    hnucleo_Spi.Init.CLKPhase 					= SPI_PHASE_1EDGE;
    hnucleo_Spi.Init.CLKPolarity 				= SPI_POLARITY_LOW;
    hnucleo_Spi.Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
    hnucleo_Spi.Init.CRCPolynomial 			=	7;
    hnucleo_Spi.Init.DataSize 					= SPI_DATASIZE_8BIT;
    hnucleo_Spi.Init.FirstBit 					= SPI_FIRSTBIT_MSB;
    hnucleo_Spi.Init.NSS 								= SPI_NSS_SOFT;
    hnucleo_Spi.Init.TIMode 						= SPI_TIMODE_DISABLE;
    hnucleo_Spi.Init.Mode 							= SPI_MODE_MASTER;
    
    SPIx_MspInit();
    HAL_SPI_Init(&hnucleo_Spi);
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  DataIn: value to be written
  * @param  DataOut: read value
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength){
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) DataIn, DataOut, DataLength, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write an amount of data to device
  * @param  DataIn: value to be written
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength){
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hnucleo_Spi, DataIn, DataLength, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
void SPIx_Write(uint8_t Value){
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;

  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) &Value, &data, 1, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
void SPIx_Error (void){
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi);
  
  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}
