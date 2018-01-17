#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_hal_def.h"
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

/****************************** Macro Definition ******************************/
#define LED_RUNNING_PERIOD		1000
#define FINGER_RX_TIMEOUT     30
#define TO_SERVER_HEADER_1		0x5A
#define TO_SERVER_HEADER_2		0x5A
#define RX_FRAME_HEADER_1			0xA5
#define RX_FRAME_HEADER_2 		0xA5

#define UPDATE_TIME_CMD				0x02		
#define UPDATE_REQ_CMD				0x03
#define UPDATE_CONTENT_CMD		0x13
#define SET_SLEEP_CMD					0x23
#define UPLOG_TYPE_CARD				0x02
#define UPLOG_TYPE_PW					0x01
#define UPLOG_TYPE_FINGER			0x03

enum SYS_FSM{ 
	SYS_IDLE, 
	POWER_ON, 
	SLEEP_MODE, 
	UPDATE_LIB
};

enum UPDATE_STATUS{ 
	ASK_UPDATE, 
	UPDATING, 
	UPDATED
};

typedef struct {
	enum SYS_FSM sys_fsm;
	uint8_t times;
}SYS_STRUCT;

typedef struct{
	uint8_t user_name[4];
	uint8_t user_pw[10];
	uint8_t user_card[8];
	uint16_t user_finger;
	uint8_t opType;	
}USER_MAG;
/****************************** Function Declaration ******************************/
SYS_STRUCT sys_para = {SYS_IDLE,  0 };
volatile uint16_t node_ID = 0x0001;
UPDOWNLOADBUF up_download_finger;
USER_MAG user_mag = {0, 0};
uint32_t sysIdle_timeTick;
extern Rx_para usart1_rx_flag, usart2_rx_flag;
extern UART_HandleTypeDef UartHandle1, UartHandle2;
extern LOCK_COM_STRUCT lock_com;

void get_system_time(void);
void update_lib(void);
void finger_OpSta(void);
void process_fingerRxBuffer(void);
void process_serverRxBuffer(void);
void server_send(uint8_t* data, uint16_t data_len, uint8_t cmd);
void process_lockUnitRxBuffer(void);
void excute_update_node(uint8_t*data, uint8_t data_len);
void enter_sleep(void);
void sysIdle_timeout(void);
void update_sysIdle_tick(void);

void SystemClock_Config(void){
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  __PWR_CLK_ENABLE();  
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100); //10ms 

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void initDrivers(void){
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
	__enable_irq();
}

uint8_t flash_running_led(){
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
void enter_sleep(void){
		//todo
	HAL_RCC_DeInit();
	 /* Disable the SysTick timer */
  SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk);

	gpio_reInit_before_stop();
	
	/* Enable Power Interface clock */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	LED_OFF( LED_NO_0 );
	usart2_rx_enbale_int();
	init_finger_touch_int();
	powerOff_LoRa();
	powerOff_finger();
	__HAL_RCC_PWR_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOB_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOC_CLK_SLEEP_ENABLE();
	__HAL_RCC_PWR_CLK_SLEEP_ENABLE();
	__HAL_RCC_SYSCFG_CLK_SLEEP_ENABLE();
	enter_StopMode();	
	initDrivers();	
}

void update_sysIdle_tick(void){
	sysIdle_timeTick = local_ticktime();
	sys_para.sys_fsm = SYS_IDLE;
}

void sysIdle_timeout(void){	
	if(timeout(sysIdle_timeTick, 10000)){
		sys_para.sys_fsm = SLEEP_MODE;
	}
}

int main (){	
	initDrivers();
	
	node_ID = (uint16_t)( HAL_GetDEVID() & 0xFFFF );
	usart1_rx_flag.rx_status = 0;
	usart2_rx_flag.rx_status = 0;
	finger.pkt_num = 0;
	finger.character_size = 0;
	memset(finger.character_buf, 0, CHARACTER_BUF_SIZE);
	finger.command = NO_CMD;	
	finger_fsm.fsm = MATCH_LOCAL;
	update_sysIdle_tick();
	
	sys_para.sys_fsm = POWER_ON;		//test to del
	//finger_fsm.sta = DELCHAR_STA;	//test to del
	
	while(1){
		flash_running_led();
		switch(sys_para.sys_fsm){
			case SYS_IDLE:
				sysIdle_timeout();
				break;
			case POWER_ON:
				get_system_time();
				break;
			case UPDATE_LIB:
				update_lib();
				break;
			case SLEEP_MODE:				
				//enter_sleep(); //test to add
				break;
			default: 
				break;
		}
		
		if(finger_fsm.fsm != FSM_IDLE){
			if(finger_fsm.sta != STA_IDLE){	
				powerOn_finger();
				finger_OpSta();
			}else{
				powerOff_finger();
			}			
		}
		
		if(lora_data.lora_data_arrived == 1){
			process_serverRxBuffer();
			lora_data.lora_data_arrived = 0;
		}		
				
		if((usart2_rx_flag.rx_status ==  1) && timeout(usart2_rx_flag.rx_timeTick, FINGER_RX_TIMEOUT)){			
			usart2_rx_flag.rx_status = 0;
			process_lockUnitRxBuffer();
		}
			
		if((usart1_rx_flag.rx_status ==  1) && timeout(usart1_rx_flag.rx_timeTick, FINGER_RX_TIMEOUT)){
			usart1_rx_flag.rx_status = 0;
			process_fingerRxBuffer();	
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_11){		
		SX1278_Interupt();
		HAL_NVIC_EnableIRQ((IRQn_Type)EXTI0_1_IRQn);
	}
	else if(GPIO_Pin == GPIO_PIN_0){
		update_sysIdle_tick();
		if((finger_fsm.sta == STA_IDLE)&&(finger_fsm.fsm != FSM_IDLE)){
			finger_fsm.sta = GETIMG1_STA;
			finger_com.cmd_retry_times = 0;
			finger_com.response_ack_tickTime = local_ticktime();
		}
	}else if( GPIO_Pin == GPIO_PIN_3 ){
		update_sysIdle_tick();
	}
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

#define FINGER_MAX_CMD_RETYR_TIMES  5
void finger_OpSta(void){
	if(finger_cmd_wait_timeOut(finger_fsm.sta) ){
		if(finger_com.cmd_retry_times < FINGER_MAX_CMD_RETYR_TIMES) {
			switch(finger_fsm.sta){
				case GETIMG1_STA:
				case GETIMG2_STA:
					sendCmd_getImg();
					break;
				case GENCHAR1_STA:
					sendCmd_genChar(FINGER_BUF_1_ID);
					break;				
				case STORECHAR_STA:
					sendCmd_store(FINGER_BUF_1_ID, finger_lib.pageID);
					break;
				case DOWNCHAR_STA:
					sendCmd_downChar(FINGER_BUF_1_ID);
					break;
				case SEARCH_STA:
					sendCmd_search(FINGER_BUF_1_ID, FINGER_LIB_START_POSITION, SEARCH_PAGES_NUM);	
					Delay_nms(300);					
					break;
				case DELCHAR_STA:
					sendCmd_deletChar(finger_lib.pageID, 1);
					//sendCmd_deletChar(0x0002, 1);
					break;
				case EMPTY_STA:
					sendCmd_empty();
				case LOADCHAR_STA:
					sendCmd_loadChar(FINGER_BUF_1_ID, 0x0004);
					break;
				case TEMPLETENUM_STA:
					sendCmd_templeteNum();
					break;
				case UPCHAR_STA:
					sendCmd_upChar(FINGER_BUF_1_ID);
				default:
					break;		
			}						
		}
		else{
			finger_com.cmd_retry_times = 0;
			finger_fsm.sta = STA_IDLE;
			if(finger.command == GETIMG || finger.command == GENCHAR){													
				sys_para.sys_fsm = SLEEP_MODE;					
			}
		}
		finger_cmd_wait_time_update();
	}
}

void process_fingerRxBuffer(void){
	uint16_t xBufferLen =  UartHandle1.RxXferSize - UartHandle1.RxXferCount;
	uint16_t startPostion = 0;
	uint16_t j = 0, m = 0;
	uint16_t frame_len = 0;
	
	/* Rx process is completed, restore huart->RxState to Ready */
	UartHandle1.RxState = HAL_UART_STATE_READY;	
	do{
		startPostion = fingerSearchFrame(UART1_RxBuffer, xBufferLen);
		if(startPostion <  xBufferLen){
				finger.character_size = UART1_RxBuffer[startPostion + 7];
				finger.character_size = (finger.character_size << 8) + UART1_RxBuffer[startPostion + 8] - 2;// 除去校验和 2字节
				m = (startPostion + 9);//确认码起始位
				for( j = 0 ; j < (finger.character_size); j++){ 
					finger.character_buf[j] = UART1_RxBuffer[m + j];
				}		
				switch(UART1_RxBuffer[startPostion + 6]){
					case PID_COMMAND:
							finger.pid = PID_COMMAND;
							finger_processCMD();
						break;
					case PID_DATA:
							finger.pid = PID_DATA;
							finger_processDATA();
						break;
					case PID_ACK:
							finger.pid = PID_ACK;
							finger_processACK();						
						break;
					case  PID_END:
							finger.pid = PID_END;
							finger_processEND();
						break;
					default: break;
			 }
			frame_len = finger.character_size + 11;	
			if(xBufferLen > frame_len){
				for( j = 0; j < ( xBufferLen - frame_len ); j++){
					UART1_RxBuffer[j] = UART1_RxBuffer[frame_len + j];
				}
				for( j = xBufferLen - frame_len; j < xBufferLen; j++){
					UART1_RxBuffer[j] = 0;
				}
				xBufferLen = xBufferLen - frame_len;
				finger.character_size = 0;
			}else{
				finger.character_size = 0;
				break;
			}
		}else{
			break;
		}
	}while(1);
	memset(finger.character_buf, 0, finger.character_size);
	finger.command = NO_CMD;	
	memset(UART1_RxBuffer, 0, UART1_RxBufferSize);	
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t*)UART1_RxBuffer, UART1_RxBufferSize);
	
}

void register_down_success_callback (void) {
	uint8_t send_buf[2] = {0x02, 0x00};
	server_send(send_buf, 2, UPDATE_CONTENT_CMD);
	finger_fsm.fsm = MATCH_LOCAL;
}

void downchar_callback (void) {
	sendData_downchar(up_download_finger.up_download_fingerModelBuff, (up_download_finger.buf_size) );	
}

void delChar_success_callback(void){
	uint8_t send_buf[2] = {0};	
	server_send(send_buf, 1, UPDATE_CONTENT_CMD);
}

void search_result_callback(uint8_t* res, uint8_t res_len){
	uint8_t send_buf[2];
	if(res[0] == 0){ //应答正确
		send_buf[0] = 0x01; //正常开
		send_cmd_to_lock( LOCK_CMD_LOCK_CON, send_buf, 1);		
		finger_lib.pageID = res[1];
		finger_lib.pageID = (finger_lib.pageID << 8) + res[2];	
	}else{ //指纹不匹配
		//todo提示错误
		sys_para.sys_fsm = SLEEP_MODE;
	}
}

#define PKT_MAX_SIZE 200
void _buf_send_to_server(uint8_t*buf, uint8_t buf_len, uint8_t cmd, uint8_t pkt_no){
	uint8_t send_buf[210] = {TO_SERVER_HEADER_1, TO_SERVER_HEADER_2};
	uint16_t temp_sumCheck = 0;
	uint8_t i;
	
	temp_sumCheck = 0;
	send_buf[2] = buf_len + 7; 
	temp_sumCheck += send_buf[2];
	send_buf[3] = (uint8_t)(node_ID >> 8); 
	temp_sumCheck += send_buf[3];
	send_buf[4] = (uint8_t)node_ID; 
	temp_sumCheck += send_buf[4];
	send_buf[5] = cmd; 
	temp_sumCheck += send_buf[5];
	for( i = 0; i < buf_len; i++){
		send_buf[i + 6] = buf[i];
		temp_sumCheck += send_buf[i + 6];
	}
	send_buf[i + 6] = pkt_no;
	temp_sumCheck += send_buf[i + 6];
	send_buf[i + 7] = (uint8_t)(temp_sumCheck >> 8);
	send_buf[i + 8] = (uint8_t)(temp_sumCheck);
	Lora_Send(send_buf, send_buf[2] + 2);
}

void server_send(uint8_t* data, uint16_t data_len, uint8_t cmd){
	uint8_t i,  pkt_num, pkt_remain, pkt_no;
	uint8_t* p = data;
	
	if(data_len > PKT_MAX_SIZE){
		pkt_num = data_len / PKT_MAX_SIZE;
		pkt_remain = data_len % PKT_MAX_SIZE;
		pkt_no = 0;
		for(i = 0; i < pkt_num; i++){
			pkt_no++;
			_buf_send_to_server(p, PKT_MAX_SIZE, cmd, pkt_no);	
			p += PKT_MAX_SIZE;
		}
		Delay_nms(500);
		_buf_send_to_server(p, pkt_remain, cmd, 0);
	}else{
		_buf_send_to_server(data, data_len, cmd, 0);
	}	
}

void process_serverRxBuffer(void){			
	if((lora_data.lora_recv_data_buf[0] == RX_FRAME_HEADER_1) && (lora_data.lora_recv_data_buf[1] == RX_FRAME_HEADER_1)){
		if(lora_data.lora_recv_data_buf[2] == (lora_data.lora_recv_data_buf_len - 2)){
			//sum crc check todo
			if( (lora_data.lora_recv_data_buf[3] == (node_ID >> 8) ) && 
					(lora_data.lora_recv_data_buf[4] == (uint8_t)node_ID ) ){
				switch(lora_data.lora_recv_data_buf[5]){
					case UPDATE_TIME_CMD : // update system time 
						send_cmd_to_lock(LOCK_CMD_CAL_TIME, &lora_data.lora_recv_data_buf[6], 6);	
						update_sysIdle_tick();					
						break;
					case UPDATE_CONTENT_CMD:
						excute_update_node(&lora_data.lora_recv_data_buf[6], lora_data.lora_recv_data_buf_len - 8);//包括子CMD+data+frameNum
						update_sysIdle_tick();
					break;
					case SET_SLEEP_CMD:
						if(lora_data.lora_recv_data_buf[6] == 0){							
							sys_para.sys_fsm = SLEEP_MODE;
						}
						break;
					default:
						sys_para.sys_fsm = SLEEP_MODE;
						break;
				}
			}
		}else{
			sys_para.sys_fsm = SLEEP_MODE;
		}
	}else{
		sys_para.sys_fsm = SLEEP_MODE;
	}
	memset(lora_data.lora_recv_data_buf, 0, lora_data.lora_recv_data_buf_len);
	lora_data.lora_recv_data_buf_len = 0;
}

void lock_next_op(union SchdParameter value){
	uint8_t send_buf[22] = {0};
	if(value.data == 1){
		memcpy(send_buf, user_mag.user_name, 4);
		send_buf[4] = 0;
		send_cmd_to_lock(LOCK_CMD_ADD_USER, send_buf, 5);
	}
	else if(value.data == 2){//添加密钥
		memcpy(send_buf, user_mag.user_name, 4);		
		memcpy(&send_buf[10], user_mag.user_pw, 10);
		send_buf[9] = 0x01; //密码类型
		send_buf[20]= 0;
		send_cmd_to_lock(LOCK_CMD_ADD_PW, send_buf, 21);	
	}
}

#define LOCK_POWER_ON   0x01
#define LOCK_DOOR_BELL	0x12
void process_lockUnitRxBuffer(void){
	static uint8_t rx_times = 0;
	uint8_t rx_dataLen = 0;
	
	rx_dataLen = UartHandle2.RxXferSize - UartHandle2.RxXferCount;
	UartHandle2.RxState = HAL_UART_STATE_READY;	
	if(UART2_RxBuffer[0] == 0xF5){		
		if((UART2_RxBuffer[3] + 4) == rx_dataLen){
			//sum check
			if(UART2_RxBuffer[4] == LOCK_UP_STA_NOTIFY){ //通知
				switch(UART2_RxBuffer[5]){
					case LOCK_POWER_ON://power on
						sys_para.sys_fsm = POWER_ON;
						break;
					case LOCK_DOOR_BELL: // door bell
						sys_para.sys_fsm = SLEEP_MODE;
						break;
					default:
						sys_para.sys_fsm = SLEEP_MODE;
						break;
				}
			}else if(UART2_RxBuffer[4] == LOCK_UP_OP_LOG){//开锁记录上传
				rx_times++;
				if(rx_times == 3){
					rx_times = 0;					
					if(UART2_RxBuffer[18] == 0){ //指纹开锁
						UART2_RxBuffer[17] = UPLOG_TYPE_FINGER; 			//figner unlock type
						UART2_RxBuffer[18] = (uint8_t)(finger_lib.pageID >> 8);
						UART2_RxBuffer[19] = (uint8_t)finger_lib.pageID;											
						server_send(&UART2_RxBuffer[17], 8, LOCK_UP_OP_LOG);
					}else{
						UART2_RxBuffer[4] = UART2_RxBuffer[18]; // type : pw | card
						// 5 6 7 8 用户名
						UART2_RxBuffer[9] = UART2_RxBuffer[20];//
						UART2_RxBuffer[10] = UART2_RxBuffer[21];
						UART2_RxBuffer[11] = UART2_RxBuffer[22];
						UART2_RxBuffer[12] = UART2_RxBuffer[23];
						UART2_RxBuffer[13] = UART2_RxBuffer[24];											
						server_send(&UART2_RxBuffer[4], 10, LOCK_UP_OP_LOG);
					}
					update_sysIdle_tick();
				}
			}else if(UART2_RxBuffer[4] == LOCK_CMD_LOCK_CON){//锁开关应答
				if(UART2_RxBuffer[5] == 00){
					//todo
				}else{
					//todo 
					sys_para.sys_fsm = SLEEP_MODE;
				}					
			}else{
				switch(lock_com.cmd_type){
					case LOCK_CMD_CAL_TIME: //更新时间
						server_send(&UART2_RxBuffer[4], UART2_RxBuffer[3], UPDATE_TIME_CMD);
						if(UART2_RxBuffer[4] == 0){
							sys_para.sys_fsm = UPDATE_LIB;
						}	
						break;
					case LOCK_CMD_ADD_USER:
							if(user_mag.opType == LOCK_CMD_CHANGE_PW){
								Schd_After_Int(300, lock_next_op, 2);								
								break;
							}								
					case LOCK_CMD_ADD_PW:
							if(user_mag.opType == LOCK_CMD_CHANGE_PW){
								user_mag.opType = 0;
							}
					case LOCK_CMD_DEL_USER:
						if(user_mag.opType == LOCK_CMD_CHANGE_PW){
							Schd_After_Int(300, lock_next_op, 1);							
							break;
						}						
					case LOCK_CMD_DEL_PW:
					case LOCK_CMD_CHANGE_USER_INFO:
					case LOCK_CMD_CHANGE_PW:
					case LOCK_CMD_INIT:
						server_send(&UART2_RxBuffer[4], UART2_RxBuffer[3], UPDATE_CONTENT_CMD);
						update_sysIdle_tick();
						break;
					default: 
						sys_para.sys_fsm = SLEEP_MODE;
						break;
				}
			}
		}	
	}else{
		sys_para.sys_fsm = SLEEP_MODE;
	}
	lock_com.cmd_type = LOCK_CMD_IDLE;
	lock_com.rx_state = RX_IDLE;
	HAL_UART_Receive_IT(&UartHandle2, (uint8_t*)UART2_RxBuffer, UART2_RxBufferSize);
}

#define GET_TIME_WAIT   3000
#define MAX_TRY_TIMES   6
void get_system_time(void){
	static uint32_t getTime_Tick;
	static uint8_t try_times;
	uint8_t send_buf[2] = {0x01};
	if(timeout(getTime_Tick, GET_TIME_WAIT)) {
		if(try_times < MAX_TRY_TIMES){
			server_send(send_buf, 1, UPDATE_TIME_CMD);
			try_times++;
		}else{
			sys_para.sys_fsm = SLEEP_MODE;
		}
		getTime_Tick = local_ticktime();
	}
}
#define MAX_GET_TIMES 5
#define GET_TIME_GAP  5000
void update_lib(void){
	uint8_t send_buf[2] = {0};
	static uint32_t time_tick;
	
	if(sys_para.times < MAX_GET_TIMES){
		if(timeout(time_tick, GET_TIME_GAP)){
			server_send(send_buf, 1, UPDATE_REQ_CMD);
			time_tick = local_ticktime();
		}
		sys_para.times++;
	}else{
		sys_para.sys_fsm = SLEEP_MODE;
	}
}

#define USER_NAME_LEN  4
void excute_update_node(uint8_t*data, uint8_t data_len){	
	uint8_t i;
	uint8_t send_buf[23] = {0};
	switch(data[0]){
		case LOCK_CMD_ADD_USER:	
			memcpy(send_buf, &data[1], 4);
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_ADD_USER, send_buf, 5);			
			break;
		case LOCK_CMD_ADD_PW:
			memcpy(send_buf, &data[1], 4);
			//send_buf[4-8] = 0;
			send_buf[9] = 0x01; //密码类型
			memcpy(&send_buf[10], &data[1], 10);//密码 = username+ pw
			send_buf[20]= 0;
			send_cmd_to_lock(LOCK_CMD_ADD_PW, send_buf, 21);			
			break;
		case LOCK_CMD_DEL_USER:
			memcpy(send_buf, &data[1], 4); //user name
			send_buf[4] = 0;
			send_cmd_to_lock(data[0], send_buf, 5);		
			if( (data[5]!=0xFF) && (data[6] != 0xFF)){
				finger_lib.pageID = data[5];
				finger_lib.pageID = (finger_lib.pageID << 8) + data[6];			
				finger_fsm.sta = DELCHAR_STA;
				finger_com.cmd_retry_times = 0;		
			}
		case LOCK_CMD_DEL_PW:
			memcpy(send_buf, &data[1], 4);
			//send_buf[4-8] = 0;
			send_buf[9] = 0x01; //密码类型
			memcpy(&send_buf[10], &data[1], 10);//密码 = username+ pw
			send_buf[20]= 0;			
			send_cmd_to_lock(LOCK_CMD_DEL_PW, send_buf, 17);
			break;
		case LOCK_CMD_CHANGE_USER_INFO:
			break;
		case LOCK_CMD_CHANGE_PW:
			user_mag.opType = LOCK_CMD_CHANGE_PW;
			memcpy(user_mag.user_name, &data[1], 4);
			memcpy(user_mag.user_pw, data, 10);
			memcpy(send_buf, user_mag.user_name, 4);
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_DEL_USER, send_buf, 5);
			break;
		case LOCK_CMD_INIT:			
			break;
		case ADD_FINGER:	//add finger
			if( data[data_len - 1] == 0x00 ){//帧序号
				finger_lib.pageID = data[data_len - 3];
				finger_lib.pageID = ( finger_lib.pageID << 8 ) + data[data_len - 2];
				for( i = 0; i < (data_len - 4); i++ ) {//除去update_content_CMD + frameNum  + flash ID 2bytes
					up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = data[i+1];
				}
				finger_fsm.fsm = REGISTER_DOWN;
				finger_fsm.sta = DOWNCHAR_STA;
			}else {	
				if( data[data_len - 1] == 0x01 ){ //第一帧数据时，clear buffer: up_download_finger
					up_download_finger.buf_size = 0;
				}
				for( i = 0; i < (data_len - 2); i++ ) {//除去update_content_CMD + frameNum 
					up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = data[i+1];
				}
			}
			break;
		case DEL_FINGER:	//delete finger
			finger_lib.pageID = data[1];
			finger_lib.pageID = (finger_lib.pageID << 8) + data[2];			
			finger_fsm.sta = DELCHAR_STA;
			finger_com.cmd_retry_times = 0;
			break;
		default:
			break;
	}
}
