#include "finger.h"
#include "usart.h"
#include "string.h"
#include "delay.h"
#include "process.h"


UPDOWNLOADBUF up_download_finger;
FINGER_PARA finger;
FINGER_FSM_STRUCT finger_fsm = {FSM_IDLE, STA_IDLE};
FINGER_COMMUNICATION finger_com;
FINGER_LIB finger_lib = {FINGER_LIB_START_POSITION};
extern UPDOWNLOADBUF up_download_finger;


uint8_t finger_cmd_wait_timeOut(enum Finger_Register sta){
	if((sta == UPCHAR_STA) ||sta == DOWNCHAR_STA ){
		return timeout(finger_com.response_ack_tickTime, FIGNER_COM_DATA_TIME);
	}else{
		return timeout(finger_com.response_ack_tickTime, FINGER_COM_ACK_TIME);
	}
}

void finger_cmd_wait_time_update(void){
	finger_com.response_ack_tickTime = local_ticktime();
}

void init_finger_touch_int(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStructure.Pin   = GPIO_PIN_0;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Mode  = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );
	
  HAL_NVIC_SetPriority((IRQn_Type)EXTI0_1_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI0_1_IRQn);
}

uint8_t send_to_finger(enum FINGER_PID pid, uint8_t* data, uint16_t data_len){
	uint16_t sum_check;
	TMsg msg;
	uint8_t j;
	
	if(data_len > TMsg_MaxLen)
		return 1;
	
	if(pid == PID_COMMAND){
		finger_com.cmd_retry_times++;
	}
	msg.Len = data_len + 11;
	msg.Data[0] = 0xEF;
	msg.Data[1] = 0x01;
	msg.Data[2] = 0xFF;
	msg.Data[3] = 0xFF;
	msg.Data[4] = 0xFF;
	msg.Data[5] = 0xFF;
	msg.Data[6] = (uint8_t) pid;
	sum_check = pid;
	sum_check += (data_len + 2);
	msg.Data[7] = (uint8_t)((data_len + 2) >> 8);
	msg.Data[8] = (uint8_t)(data_len + 2);

	for(j = 0; j < data_len; j++){
		msg.Data[9+j] = data[j];
		sum_check += data[j];
	}
	msg.Data[9+j] = (uint8_t)(sum_check >> 8);
	msg.Data[10+j] = (uint8_t)(sum_check);
	UART_SendMsg(&msg, USART1);
	return 0;
}

void sendCmd_getImg(void){
	uint8_t cmd_buf[2] = {0x01};
	
	finger.pid = PID_COMMAND;
	finger.command = GETIMG;
	send_to_finger(PID_COMMAND, cmd_buf, 1);
}

void sendCmd_genChar(uint8_t bufID){
	uint8_t cmd_buf[2] = {0x02};
	
	finger.pid = PID_COMMAND;
	finger.command = GENCHAR;
	cmd_buf[1] = bufID;
	send_to_finger(PID_COMMAND, cmd_buf, 2);
}

void sendCmd_regModel(void){
	uint8_t cmd_buf[2] = {0x05};
	
	finger.pid = PID_COMMAND;
	finger.command = REGMODEL;
	send_to_finger(PID_COMMAND, cmd_buf, 1);
}

// bufid, charBuffer1 charBuffer2, 1hºÍ2h
void sendCmd_upChar(uint8_t bufID){
	uint8_t cmd_buf[2] = {0x08};
	
	finger.pid = PID_COMMAND;
	finger.command = UPCHAR;
	cmd_buf[1] = bufID;
	send_to_finger(PID_COMMAND, cmd_buf, 2);
}

// bufid, charBuffer1 charBuffer2, 1hºÍ2h
void sendCmd_downChar(uint8_t bufID){
	uint8_t cmd_buf[2] = {0x09};
	
	finger.pid = PID_COMMAND;
	finger.command = DOWNCHAR;
	cmd_buf[1] = bufID;
	send_to_finger(PID_COMMAND, cmd_buf, 2);
}
#define FINGER_MAX_DATA_SIZE 128
void sendData_downchar(uint8_t* buf, uint16_t buf_len){
	uint8_t pkt_num, pkt_remain;
	uint8_t i;
	uint8_t* p = buf;
	//__disable_irq();
	pkt_num = buf_len / FINGER_MAX_DATA_SIZE;
	pkt_remain = buf_len % FINGER_MAX_DATA_SIZE;
	for( i = 0; i < pkt_num; i++){
		send_to_finger(PID_DATA, p, FINGER_MAX_DATA_SIZE);
		p += FINGER_MAX_DATA_SIZE;		
	}
	if(pkt_remain != 0){
		send_to_finger(PID_DATA, p, pkt_remain);
	}
	//__enable_irq();
}

void sendCmd_store(uint8_t bufID, uint16_t pageID){
	uint8_t cmd_buf[4] = {0x06};
	
	finger.pid = PID_COMMAND;
	finger.command = 	STORE;
	cmd_buf[1] = bufID;
	cmd_buf[2] = (uint8_t)(pageID >> 8);
	cmd_buf[3] = (uint8_t)pageID;
	send_to_finger(PID_COMMAND, cmd_buf, 4);
}

void sendCmd_loadChar(uint8_t bufID, uint16_t pageID){
	uint8_t cmd_buf[4] = {0x07};
	
	finger.pid = PID_COMMAND;
	finger.command = 	LOADCHAR;
	cmd_buf[1] = bufID;
	cmd_buf[2] = (uint8_t)(pageID >> 8);
	cmd_buf[3] = (uint8_t)pageID;	
	send_to_finger(PID_COMMAND, cmd_buf, 4);
}

void sendCmd_deletChar(uint16_t pageID, uint16_t deletNum){
	uint8_t cmd_buf[5] = {0x0c};
	
	finger.pid = PID_COMMAND;
	finger.command = 	DELETCHAR;
	cmd_buf[1] = (uint8_t)(pageID >> 8);
	cmd_buf[2] = (uint8_t)pageID;
	cmd_buf[3] = (uint8_t)(deletNum >> 8);
	cmd_buf[4] = (uint8_t) deletNum;
	send_to_finger(PID_COMMAND, cmd_buf, 5);
}

void sendCmd_empty(void){
	uint8_t cmd_buf[2] = {0x0d};
	
	finger.pid = PID_COMMAND;
	finger.command = 	EMPTY;
	send_to_finger(PID_COMMAND, cmd_buf, 1);
}

void sendCmd_match(void){
	uint8_t cmd_buf[2] = {0x03};
	
	finger.pid = PID_COMMAND;
	finger.command = 	MATCH_CMD;
	send_to_finger(PID_COMMAND, cmd_buf, 1);
}

void sendCmd_search(uint8_t bufID, uint16_t startPage, uint16_t pageNum){
	uint8_t cmd_buf[6] = {0x04};
		
	finger.pid = PID_COMMAND;
	finger.command = 	SEARCH_CMD;
	cmd_buf[1] = bufID;
	cmd_buf[2] = (uint8_t)(startPage >> 8);
	cmd_buf[3] = (uint8_t)startPage;
	cmd_buf[4] = (uint8_t)(pageNum >> 8);
	cmd_buf[5] = (uint8_t) pageNum;	
	send_to_finger(PID_COMMAND, cmd_buf, 6);
}

void sendCmd_templeteNum(void){
	uint8_t cmd_buf[2] = {0x1D};
	
	finger.pid = PID_COMMAND;
	finger.command = 	READTEMPLETENUM;
	send_to_finger(PID_COMMAND, cmd_buf, 1);
}

//void sendCmd_setSysPara(uint8_t paraNum, uint8_t para){
//	uint8_t cmd_buf[3] = {0x0C};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = 	SETSYSPARA;
//	cmd_buf[1] = paraNum;
//	cmd_buf[2] = para;
//	send_to_finger(PID_COMMAND, cmd_buf, 3);
//}

//void sendCmd_readSysPara(void){
//	uint8_t cmd_buf[2] = {0x0F};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = READSYSPARA;
//	send_to_finger(PID_COMMAND, cmd_buf, 1);
//}	
//page: 0~15
//content size :32bytes
//void sendCmd_writeNotepad(uint8_t notePageNum, uint8_t* content){
//	uint8_t cmd_buf[34] = {0x18};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = WRITENOTEPAD;
//	cmd_buf[1] = notePageNum;
//	for(uint8_t i = 0; i < 32; i++){
//		cmd_buf[i+2] = content[i];
//	}
//	send_to_finger(PID_COMMAND, cmd_buf, sizeof(cmd_buf));
//}
//void sendCmd_readNotepad(uint8_t notePageNum){
//	uint8_t cmd_buf[2] = {0x19};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = READNOTEPAD;
//	cmd_buf[1] = notePageNum;
//	send_to_finger(PID_COMMAND, cmd_buf, sizeof(cmd_buf));
//}

//void sendCmd_readFpflash(uint8_t notePageNum){
//	uint8_t cmd_buf[2] = {0x1F};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = 	READFPFLASH;
//	cmd_buf[1] = notePageNum;
//	send_to_finger(PID_COMMAND, cmd_buf, sizeof(cmd_buf));
//}

//void sendCmd_readInfo(uint8_t notePageNum){
//	uint8_t cmd_buf[2] = {0x47};
//	
//	finger.pid = PID_COMMAND;
//	finger.command = READINFO;
//	cmd_buf[1] = notePageNum;
//	send_to_finger(PID_COMMAND, cmd_buf, 2);
//}

uint16_t fingerSearchFrame(uint8_t* data, uint16_t len) {
	uint16_t i;
	
	if(len < 11)
			return len;
	
	for( i = 0; i <  len; i++){
		if( (data[i] == 0xEF) && (data[i+1] == 0x01) ) //no compare with module ID 0xFFFFFFFF
			break;
	}
	
	if(i >= len)
		return len;
	
	return i;
}

void finger_processCMD(void){
	//todo
}

void finger_processACK(void){
	switch(finger.command){
		case GETIMG:
			if(finger.character_buf[0] == 0){ 
				finger_com.cmd_retry_times = 0;				
				if(finger_fsm.sta == 	GETIMG1_STA){
					finger_fsm.sta = GENCHAR1_STA;										
				}
				else{
					finger_fsm.sta = STA_IDLE;
				}
			}else{
				finger_callBack(MATCH_LOCAL_FSM, finger.character_buf);
			}
			break;
		case GENCHAR:			
			if(finger.character_buf[0] == 0){ 
				finger_com.cmd_retry_times = 0;							
				if(finger_fsm.sta == GENCHAR1_STA) {
					if(finger_fsm.fsm == MATCH_LOCAL_FSM){
						finger_fsm.sta = SEARCH_STA;										
						finger_cmd_wait_time_update();			
					}else{
						finger_fsm.sta = STA_IDLE;							
					}
				}
			}else{
				finger_callBack(MATCH_LOCAL_FSM, finger.character_buf);
			}
			break;		
		case STORE:			
			if(finger.character_buf[0] == 0){				
				finger_com.cmd_retry_times = 0;				
				if(finger_fsm.fsm == REGISTER_DOWN_FSM){
					finger_callBack(REGISTER_DOWN_FSM, 0);				
				}else{
					finger_fsm.sta = STA_IDLE;
				}
			}			
			break;
		case SEARCH_CMD:
			if(finger.character_size == 5){
				finger_callBack(MATCH_LOCAL_FSM, finger.character_buf);
			}
			break;						
		case DOWNCHAR	:			
			if(finger.character_buf[0] == 0){					
				sendData_downchar(up_download_finger.up_download_fingerModelBuff, (up_download_finger.buf_size) );				
				finger_com.cmd_retry_times = 0;				
				finger_fsm.sta = STORECHAR_STA;
			}else{
				finger_fsm.sta = STA_IDLE;
			}
			break;
		case DELETCHAR:
			finger_callBack(DELETE_CHAR_FSM, finger.character_buf);					
			break;
		case EMPTY :	
			finger_callBack(EMPTY_LIB_FSM, finger.character_buf);			
			break;
		default:
			break;
	}
}

void finger_processEND(void){
	if(up_download_finger.receive_data_flag == 1){
		if(finger.pkt_num++ <= 6 ){
			for(uint16_t i = 0; i < finger.character_size; i++){
				up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = finger.character_buf[i];				
			}
			if(finger.pkt_num == 6){				
				//finger_module_upper_callBack();
			}
		}
	}
}

void finger_processDATA(void){
	if(up_download_finger.receive_data_flag == 1){
		finger.pkt_num++;
		if(finger.pkt_num <= 6 ){
			for(uint16_t i = 0; i < finger.character_size; i++){
				up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = finger.character_buf[i];				
			}
			if(finger.pkt_num == 6){
				finger_fsm.sta = STA_IDLE;
			}
		}
	}
}

