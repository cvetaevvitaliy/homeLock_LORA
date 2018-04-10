#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_hal_def.h"
#include "main.h"
#include "led.h"
#include "spi.h"
#include "lowpower.h"
#include "sx1276.h"
#include "usart.h"
#include "finger.h"
#include "lock.h"
#include "schd.h"
#include "globalvar.h"
#include "delay.h"
#include "string.h"
#include "temp.h"
#include "process.h"
#include "lora_com.h"
#include "stdlib.h"
#include "rtc_alarm.h"

/****************************** Variables Definition ******************************/
SYS_STRUCT sys_para = {SYS_IDLE,  0 ,{0}};
USER_MAG user_mag = {0, 0};
uint8_t bell_update = 0;
uint8_t alarm_valid = 0;
/****************************** Function Definition ******************************/
static void SystemClock_Config(void){
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;
  
  __PWR_CLK_ENABLE();  
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    while(1);
  }
	/**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100); //10ms 

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	 /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void initDrivers(void){
	__disable_irq();
  HAL_Init();
  SystemClock_Config();
	init_led();
	SPIx_Init();
	init_PowerControl_Configuration();
	USARTConfig();
	powerOn_LoRa();
	Lora_Init();
	init_finger_touch_int();
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);
	__enable_irq();
}

static uint8_t flash_running_led(){
	static uint8_t	ledStatus	= 0;
	static uint32_t runningTick	= 0;
   
	if ( timeout( runningTick, LED_RUNNING_PERIOD ) ){			
		if ( ledStatus == 0 ){
			LED_ON( LED_NO_0 );
			ledStatus = 1;
		} else {
			LED_OFF( LED_NO_0 );
			ledStatus = 0;
		}
		runningTick = local_ticktime();
		return 1;
	}
	return 0;
}
static uint32_t GetLockCode(void){
	uint32_t CpuID[3];
	uint32_t Lock_Code;

	CpuID[0] = *(uint32_t*)(0x1ff80050);
	CpuID[1] = *(uint32_t*)(0x1ff80054);
	CpuID[2] = *(uint32_t*)(0x1ff80064);
	
	Lock_Code = CpuID[0] + CpuID[1] + CpuID[2];
	return (Lock_Code);
}
static void sysIdle_timeout(void){	
	if(timeout(sys_para.sysIdle_timeTick, SYS_IDLE_TIME)){
		sys_para.sysIdle_timeTick = local_ticktime();
		sys_para.sys_fsm = SLEEP_MODE;
	}
}

void update_sysIdle_tick(void){
	sys_para.sysIdle_timeTick = local_ticktime();
	sys_para.sys_fsm = SYS_IDLE;
}

static void init_para_variables(void){
	init_lock_com();
	init_lora_txDataBuf(&lora_txBuf);
	bell_update = 0;
	user_mag.opType = 0;
	
	lockUnit.nodeID = GetLockCode();
	usart1_rx_flag.rx_status = 0;
	usart2_rx_flag.rx_status = 0;
	finger.pkt_num = 0;
	finger.character_size = 0;
	memset(finger.character_buf, 0, CHARACTER_BUF_SIZE);
	finger.command = NO_CMD;	
	up_download_finger.frame_num = 0;
}

extern SPI_HandleTypeDef hnucleo_Spi;

void enter_sleep(void){
	uint8_t send_buf[130] = {UPDATE_TIME_REQ};
	
	HAL_RCC_DeInit();
	  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi);
	 /* Disable the SysTick timer */
  SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk);

	LED_OFF( LED_NO_0 );
	LED_OFF( LED_NO_1);
	powerOff_LoRa();
	powerOff_finger();
	gpio_reInit_before_stop();
	usart2_rx_enbale_int();
	init_finger_touch_int();

	/* Enable Power Interface clock */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	__HAL_RCC_PWR_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOB_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOC_CLK_SLEEP_ENABLE();
	__HAL_RCC_PWR_CLK_SLEEP_ENABLE();
	__HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE();
	enter_StopMode();	
	initDrivers();
	init_para_variables();
	update_sysIdle_tick();
	if(alarm_valid == 1){
		uint8_t len = isEmptyRecord();
		uint8_t sendBufLen = sizeof(Record)*len;
		if(len != 0xFF){
			memcpy(send_buf, record, sendBufLen);
			sendToServer(send_buf, sendBufLen, LOCK_UP_OP_LOG , 1);
			delAllRecord();
		}else{
			send_buf[0] = UPDATE_TIME_REQ;
			sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1);			
		}
		alarm_valid = 0;
	}
}

void sys_fsm(void){
	switch(sys_para.sys_fsm){
		case SYS_IDLE:
			sysIdle_timeout();			
			break;
		case SLEEP_MODE:
			enter_sleep();
			break;
		default: 
			break;
	}
}
//RTC的Alarm时间设定在24 Hours的2点到4点之间的时段更新
int main (){
	
	uint16_t temp;
	initDrivers();
	init_RTC();
	init_para_variables();
	srand(lockUnit.nodeID);
	temp = rand()%101;
	setAlarm((2 + temp/60), (((temp%60)/10 << 4) + ((temp%60)&0x0F)));
	finger_fsm.fsm = FSM_IDLE;
	// wait lock startUp
	Delay_nms(4000);
	update_sysIdle_tick();
	
	while(1){
		flash_running_led();
		lora_com_fsm(0);		
		sys_fsm();
		
		if(bell_update == 1){
			if(timeout(sys_para.sysIdle_timeTick, 5000)){				
//				uint8_t voice_data[2] = {S_SET_FAILED, 0};//todo语音去掉，修改为LED闪烁
//				send_cmd_to_lock(LOCK_CMD_VOICE_BROADCAST, voice_data, 1 );	
				powerOn_finger();//1次
				Delay_nms(300);
				powerOff_finger();
				Delay_nms(300);
				powerOn_finger();//2次
				Delay_nms(300);
				powerOff_finger();
				Delay_nms(300);
				powerOn_finger();//3次
				Delay_nms(300);
				powerOff_finger();
				bell_update = 0;
			}		
		}
					
		//Finger ----
		if(finger_fsm.sta == STA_IDLE){
			if(finger_fsm.fsm != FSM_IDLE){
				if(power[1].status != POWERON){
					powerOn_finger();
					finger_com.cmd_retry_times = 0;
					finger_com.response_ack_tickTime = local_ticktime();
				}else{
					//指纹模块上电后无回应
					if(timeout(finger_com.response_ack_tickTime, FINGER_NO_TOUCH_TIME)){ 
						powerOff_finger();
						finger_fsm.fsm = FSM_IDLE;
						finger_com.response_ack_tickTime = local_ticktime();
					}
				}
			}else{
				if(power[1].status == POWERON){
					powerOff_finger();
				}
			}
		}else{
			finger_OpSta();
		}
		// finger Rx
		if((usart1_rx_flag.rx_status ==  1) && timeout(usart1_rx_flag.rx_timeTick, FINGER_UART_RX_TIMEOUT)){
			usart1_rx_flag.rx_status = 0;
			process_fingerRxBuffer();	
		}
		// LOCK Unit---
		if(lock_com.cmd_type != LOCK_CMD_IDLE){
			if(timeout(lock_com.timeoutTick, LOCK_ACK_TIME)){
				if(lock_com.retry_times < LOCK_MAX_RETRY_TIMES){
					lock_com.retry_times++;
					send_cmd_to_lock(lock_com.cmd_type, lock_com.last_data, lock_com.last_data_len );
				}else{
					//门锁指令失败，上传执行失败信息
					lock_cmd_timeout_callback();
					init_lock_com();
				}
				lock_com.timeoutTick = local_ticktime();
			}
		}
		// Rx
		if((usart2_rx_flag.rx_status ==  1) && timeout(usart2_rx_flag.rx_timeTick, LOCK_UART_RX_TIMEOUT)){			
			usart2_rx_flag.rx_status = 0;
			process_lockUnitRxBuffer();
		}
	}
}

//-----   Interrupt Callback function -------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_11){		
		SX1278_Interupt();
		HAL_NVIC_EnableIRQ((IRQn_Type)EXTI0_1_IRQn);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	}
	else if(GPIO_Pin == GPIO_PIN_0){			
		finger_fsm.fsm = MATCH_LOCAL_FSM;			
	}
	else if( GPIO_Pin == GPIO_PIN_3 ){ //UART Rx
		//nothing to do 
	}
	update_sysIdle_tick();	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	
	if(huart->Instance == USART2){		
		usart2_rx_flag.rx_status = 1;
		usart2_rx_flag.rx_timeTick = local_ticktime();
		update_sysIdle_tick();
	}
	if(huart->Instance == USART1){
		usart1_rx_flag.rx_status = 1;
		usart1_rx_flag.rx_timeTick = local_ticktime();
	}
}


void copy_data_to_lora_txBuf(uint8_t*buf, uint8_t buf_len, uint8_t cmd, uint8_t pkt_no, uint8_t ack){
	uint8_t send_buf[210] = {TO_SERVER_HEADER_1, TO_SERVER_HEADER_2};
	uint16_t temp_sumCheck = 0;
	uint8_t i;
	
	temp_sumCheck = 0;
	send_buf[2] = buf_len + 9; 
	temp_sumCheck += send_buf[2];
	send_buf[3] = (uint8_t)(lockUnit.nodeID >> 24); 
	temp_sumCheck += send_buf[3];
	send_buf[4] = (uint8_t)(lockUnit.nodeID >> 16); 
	temp_sumCheck += send_buf[4];
	send_buf[5] = (uint8_t)(lockUnit.nodeID >> 8); 
	temp_sumCheck += send_buf[5];
	send_buf[6] = (uint8_t)lockUnit.nodeID; 
	temp_sumCheck += send_buf[6];
	send_buf[7] = cmd; 
	temp_sumCheck += send_buf[5];
	for( i = 0; i < buf_len; i++){
		send_buf[i + 8] = buf[i];
		temp_sumCheck += send_buf[i + 8];
	}
	send_buf[i + 8] = pkt_no;
	temp_sumCheck += send_buf[i + 6];
	send_buf[i + 9] = (uint8_t)(temp_sumCheck >> 8);
	send_buf[i + 10] = (uint8_t)(temp_sumCheck);
	write_datas_to_txBuf( &lora_txBuf, send_buf, send_buf[2] + 2, ack);
}

#define PKT_MAX_SIZE 200
void sendToServer(uint8_t* data, uint16_t data_len, uint8_t cmd, uint8_t ack){
	uint8_t i,  pkt_num, pkt_remain, pkt_no;
	uint8_t* p = data;
	
		if(data_len > PKT_MAX_SIZE){
			pkt_num = data_len / PKT_MAX_SIZE;
			pkt_remain = data_len % PKT_MAX_SIZE;
			pkt_no = 0;
			for(i = 0; i < pkt_num; i++){
				pkt_no++;
				copy_data_to_lora_txBuf(p, PKT_MAX_SIZE, cmd, pkt_no, ack);	
				p += PKT_MAX_SIZE;
			}
			copy_data_to_lora_txBuf(p, pkt_remain, cmd, 0, ack);
		}else{
			copy_data_to_lora_txBuf(data, data_len, cmd, 0, ack);
		}
}
// ----    end   -----
