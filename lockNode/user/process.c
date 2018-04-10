#include "process.h"
#include "string.h"
#include "finger.h"
#include "usart.h"
#include "lock.h"
#include "main.h"
#include "schd.h"
#include "sx1276.h"
#include "lora_com.h"
#include "led.h"
#include "lowpower.h"
#include "temp.h"
#include "delay.h"
#include "rtc_alarm.h"

Record record[MAX_RECORD_NUM];
//使用FIFO的方式，删除的第一个存储位置内容，后面的信息补上；
//加入时，最新数据加在后面位置

// 返回值， 0 表示非空， 1表示空
uint8_t isEmptyRecord(void){
	uint8_t i;
	
	for(i = 0; i < MAX_RECORD_NUM; i++){
			if(record[i].updateFlag != 0)
				return (i+1);
	}
	
	return 0xFF;
}
//加入的记录数据格式：连续
// 类型1字节+指纹地址2字节+开门时间5字节+电压信息5字节
void addRecord(uint8_t* data){
	uint8_t i;
	for(i = 0; i < MAX_RECORD_NUM; i++){
		
		if(record[i].updateFlag == 0){
			record[i].updateFlag = data[0];
			record[i].fingerAddress = data[1];
			record[i].fingerAddress = ( record[i].fingerAddress << 8 ) + data[2];
			memcpy(record[i].openDoorTime , &data[3], 5);
			memcpy(record[i].batterInfo, &data[8], 5);			
			break;
		}
	}
}

void delFirstRecord(void){
	uint8_t i;
	
	record[0].updateFlag = 0;
	for(i = 1; i < MAX_RECORD_NUM; i++){
		if(record[i].updateFlag == 0)
			break;
	}
	
	if(i > 1){
		memmove(&record[0].fingerAddress, &record[1].fingerAddress, sizeof(Record)* (i - 1) );
	}
}
void delLastRecord(void){
	uint8_t i;
	
	for(i = 0; i < MAX_RECORD_NUM; i++){
		if(record[i].updateFlag == 0)
			break;
	}
	
	if(i != 0){
		memset(&record[i - 1].updateFlag, 0, sizeof(Record));
	}
}

void delAllRecord(void){
	uint8_t i;
	
	for( i = 0; i < MAX_RECORD_NUM; i++){
		if(record[i].updateFlag != 0){
			memset(&record[i].updateFlag, 0, sizeof(Record));
		}
	}	
}
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
					sendCmd_search(FINGER_BUF_1_ID, FINGER_LIB_START_POSITION, MAX_FINGER_LIB_NUM);	
					break;
				case DELCHAR_STA:					
					sendCmd_deletChar(finger_lib.pageID, 1);					
					break;
				case EMPTY_STA:
					sendCmd_empty();
					break;
				default:
					break;		
			}						
		}else{
			if(finger_fsm.fsm != MATCH_LOCAL_FSM){
				uint8_t send_buf[2] = {0};
				send_buf[0] = user_mag.opType;
				send_buf[1] = 1;
				sendToServer(send_buf, 2, UPDATE_CONTENT_CMD, 0 );
			}
			finger_com.cmd_retry_times = 0;
			finger_fsm.sta = STA_IDLE;
		}
		finger_cmd_wait_time_update();
	}
}

void finger_callBack(enum FINGER_FSM fsm, uint8_t* data){
	uint8_t send_buf[5] = {0};
	
	switch(fsm){
		case REGISTER_DOWN_FSM:
			send_buf[0] = user_mag.opType;//add finger
			send_buf[1] = 0;
			sendToServer(send_buf, 2, UPDATE_CONTENT_CMD, 1 );
			break;
		case DELETE_CHAR_FSM:	
			if(user_mag.opType == LOCK_CMD_DEL_USER){
				send_buf[0] = LOCK_CMD_DEL_USER;
			}
			if(data[0] == 0){
				send_buf[1] = 0;
			}else{//failed				
				send_buf[1] = 1;
			}
			sendToServer(send_buf, 2, UPDATE_CONTENT_CMD , 1);
			break;
		case MATCH_LOCAL_FSM:
			if(data[0] == 0){ 			//应答正确 - 正常开门
				send_buf[0] = 0x01; 	
				send_cmd_to_lock( LOCK_CMD_LOCK_CON, send_buf, 1);		
				finger_lib.pageID = data[1];
				finger_lib.pageID = (finger_lib.pageID << 8) + data[2];					
			}else{ 								//指纹错误/无效
				send_buf[0] = S_FPT_INVALID;
				send_cmd_to_lock(LOCK_CMD_VOICE_BROADCAST, send_buf, 1);
			}
			break;
		case EMPTY_LIB_FSM:
			send_buf[0] = user_mag.opType;
			if(data[0] == 0){
				send_buf[1] = 0;
			}else{
				send_buf[1] = 1;
			}
			if(user_mag.opType != SET_MANUFACTURE_CMD){
				sendToServer(send_buf, 2, UPDATE_CONTENT_CMD , 1);			
			}
			break;
		default:
			break;
	}
	finger_fsm.fsm = FSM_IDLE;
	finger_fsm.sta = STA_IDLE;
	finger_com.cmd_retry_times = 0;
	finger_com.response_ack_tickTime = local_ticktime();
}
// return 0: none; 1: ack right
static uint8_t finger_powerOn_ack(uint8_t* data, uint8_t dataLen){
	uint8_t i;
	if(dataLen > 6)
		return 0;
	
	for( i = 0; i < dataLen ; i++){
		if(data[i] == 0x55){
			return 1;
		}
	}
	return 0;
}

void process_fingerRxBuffer(void){
	uint16_t xBufferLen =  UartHandle1.RxXferSize - UartHandle1.RxXferCount;
	uint16_t startPostion = 0;
	uint16_t j = 0, m = 0;
	uint16_t frame_len = 0;
	
	/* Rx process is completed, restore huart->RxState to Ready */
	UartHandle1.RxState = HAL_UART_STATE_READY;	
	if(finger_powerOn_ack(UART1_RxBuffer, xBufferLen)){
		//change fsm stasus (1、指纹按下并比对 2、更新指纹库；3、清除指纹库）
		switch(finger_fsm.fsm){
			case MATCH_LOCAL_FSM:
				finger_fsm.sta = GETIMG1_STA;				
				break;
			case REGISTER_DOWN_FSM:
				finger_fsm.sta = DOWNCHAR_STA;
				break;
			case DELETE_CHAR_FSM:
				finger_fsm.sta = DELCHAR_STA;
				break;
			case EMPTY_LIB_FSM:
				finger_fsm.sta = EMPTY_STA;
			default:
				break;
		}
		finger_com.cmd_retry_times = 0;
	}else{
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
	}
	memset(UART1_RxBuffer, 0, UART1_RxBufferSize);	
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t*)UART1_RxBuffer, UART1_RxBufferSize);
}

//-------------   LOCK   ---------------
uint8_t lock_cmd_timeout_callback(void){
	uint8_t send_buf[10] = {0, 1, 0};
	
	switch(lock_com.cmd_type){
		case LOCK_CMD_CAL_TIME:
			send_buf[0] = UPDATE_TIME_FAILED;//更新时间失败
			sendToServer(send_buf, 1, UPDATE_TIME_CMD, 0 );
			return 0;			
		case LOCK_CMD_ADD_USER:
			send_buf[0] = LOCK_CMD_ADD_USER;		
			break;
		case LOCK_CMD_ADD_PW:
			send_buf[0] = LOCK_CMD_ADD_PW;			
			break;
		case LOCK_CMD_DEL_USER:
			send_buf[0] = LOCK_CMD_DEL_USER;			
			break;
		case LOCK_CMD_DEL_PW:
			send_buf[0] = LOCK_CMD_DEL_PW;	
			break;
		default:
			break;
	}		
	sendToServer(send_buf, 2, UPDATE_CONTENT_CMD, 0 );
	return 0;
}

void lock_voice(union SchdParameter value){
	uint8_t send_buf[2] = {S_INITIALIZING, 0};
	if(bell_update == 1){		
		send_cmd_to_lock(LOCK_CMD_VOICE_BROADCAST, send_buf, 1);
	}
}
void next_op(union SchdParameter value){
	uint8_t send_buf[22] = {0};
	switch(value.data){
		case 1: // 添加用户
			memcpy(send_buf, user_mag.user_name, 4);
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_ADD_USER, send_buf, 5);
			break;
		case 2://添加密钥
			memcpy(send_buf, user_mag.user_name, 4);		
			memcpy(&send_buf[10], user_mag.user_pw, 10);
			send_buf[9] = 0x01; 			//密码类型
			send_buf[20]= 0;
			send_cmd_to_lock(LOCK_CMD_ADD_PW, send_buf, 21);	
			break;
		case 3: //查询门锁状态
			send_cmd_to_lock(LOCK_CMD_STATUS, send_buf, 0);	
			break;
		case 4:				
			send_buf[0]= UPDATE_TIME_REQ;
			sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1);
			break;
		case 5:
			enable_doorBell();
			break;
		case 6:
			sys_para.sys_fsm = SLEEP_MODE;
			break;
		default:                                                                                                         
			break;
	}
}

uint8_t lock_rx_check(uint8_t* input, uint8_t input_len, uint8_t* output, uint8_t* output_len){
	uint8_t i, j;
	uint16_t temp_sum = 0, checkSum = 0;

	if(input_len < 5)
		return 0xFF;
	
	for( i = 0; i < (input_len - 4); i++){
		if(input[i] == 0xF5){			
			checkSum += input[i+3];
			for( j = 4; j < (input[i+3] + 4); j++){
				checkSum += input[i+j];
			}
			temp_sum = input[i+1];
			temp_sum = (temp_sum << 8) + input[i+2];
			
			if(checkSum == temp_sum){
				*output_len = input[i+3] + 4;
				memcpy(output, &input[i], *output_len );	
				return 	i;
			}
		}
	}
	
	return 0xFF;
}
uint8_t openLockType;
uint8_t openLockTime[5];
void process_lockUnitRxBuffer(void){

	uint8_t send_buf[20] = {0};
	uint8_t frame_start_position;
	uint8_t temp_rx_data[60];
	uint8_t temp_rx_data_len;
		
	UartHandle2.RxState = HAL_UART_STATE_READY;	
	frame_start_position = lock_rx_check(UART2_RxBuffer, UartHandle2.RxXferSize - UartHandle2.RxXferCount, \
													temp_rx_data, &temp_rx_data_len);
	
	if( frame_start_position != 0xFF ){
		//门锁主动通知
		if(temp_rx_data[frame_start_position+4] == LOCK_UP_STA_NOTIFY){	 
			update_sysIdle_tick();
				switch(temp_rx_data[frame_start_position+5]){
					case LOCK_POWER_ON:  //power on 						
						send_cmd_to_lock(LOCK_CMD_ACK, send_buf, 0 );
						Schd_After_Int(20, next_op, 5);																								
						break;
					case LOCK_DOOR_BELL: // door bell
						bell_update = 1;
						send_cmd_to_lock(LOCK_CMD_ACK, send_buf, 0 );
						Schd_After_Int(3000, next_op, 4);				//响铃期间执行，失败
//						Schd_After_Int(2000, lock_voice, 0);	//这两个参数的时间最优化了,取消声音	
						break;
					default:
						break;
				}		
		}
		//开锁记录上传
		else if(temp_rx_data[frame_start_position+4] == LOCK_UP_OP_LOG){
			user_mag.opType = LOCK_UP_OP_LOG;
			send_buf[0] = 0;
			send_cmd_to_lock(LOCK_UP_OP_LOG, send_buf, 1 );//对门锁的应答回应
			if(temp_rx_data[frame_start_position+18] == 0){ //指纹开锁
				
				send_buf[0] = UPLOG_TYPE_FINGER; 			//figner unlock type
				send_buf[1] = (uint8_t)(finger_lib.pageID >> 8);
				send_buf[2] = (uint8_t)finger_lib.pageID;
				//	TIME : Y_M_D_H_M
				memcpy(&send_buf[3], &temp_rx_data[frame_start_position+20], 5);
				send_buf[8] = lockUnit.batteryLevel;
				//	Battery Voltage	unit: 1uV, little edian to big edian mode
				send_buf[9] = temp_rx_data[frame_start_position+32];
				send_buf[10] = temp_rx_data[frame_start_position+31];
				send_buf[11] = temp_rx_data[frame_start_position+30];
				send_buf[12] = temp_rx_data[frame_start_position+29];
				sendToServer(send_buf, 13, LOCK_UP_OP_LOG , 1);	
				openLockType = 0;
				addRecord(send_buf);
				}else{	//卡或是密码开锁, password : 1, card: 2 
					openLockType = temp_rx_data[frame_start_position+18];
					memcpy(user_mag.user_name, &temp_rx_data[frame_start_position+5], 4);	
					memcpy(openLockTime, &temp_rx_data[frame_start_position+20], 5);
					Schd_After_Int(20, next_op, 3);
			}
			update_sysIdle_tick();
		}
		//如果是指纹开锁，锁开关应答之后获取电量信息，
		//如果面板密码或是刷卡开锁，
		else if(temp_rx_data[frame_start_position+4] == LOCK_CMD_LOCK_CON){
			openLockType = UPLOG_TYPE_FINGER;
			Schd_After_Int(20, next_op, 3);	
			if(lock_com.cmd_type != LOCK_CMD_IDLE){
				init_lock_com();
			}				
		}
		else{
			switch(lock_com.cmd_type){
				case LOCK_CMD_CAL_TIME: 
					if(temp_rx_data[frame_start_position+4] == 0){
						send_buf[0] = UPDATE_TIME_SUCCESS;//更新时间成功	
						sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1 );
					}else{
						send_buf[0] = UPDATE_TIME_FAILED;//更新时间失败
						sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1 );
					}
					break;
				case LOCK_CMD_DEL_USER: //小于FD FF为成功
					if(user_mag.opType == LOCK_CMD_CHANGE_PW){
						Schd_After_Int(500, next_op, 1);							
						break;
					}
					if(finger_lib.pageID < MAX_FINGER_LIB_NUM){
						break;
					}
					send_buf[0] = LOCK_CMD_DEL_USER;
					if(temp_rx_data[frame_start_position + 4] == 0){							
						send_buf[1] = 0;							
					}else{// failed
						send_buf[1] = 1;
					}
					sendToServer(send_buf, 2, UPDATE_CONTENT_CMD, 0 );
					update_sysIdle_tick();
					break;
				case LOCK_CMD_STATUS:
					lockUnit.batteryLevel = temp_rx_data[frame_start_position + 9];
					if(openLockType != UPLOG_TYPE_FINGER){
						send_buf[0] = openLockType;
						memcpy(&send_buf[1], (char*)user_mag.user_name, 4);
						memcpy(&send_buf[5], (char*)openLockTime, 5);
						send_buf[10] = lockUnit.batteryLevel;
						// Battery Voltage unit: 1uV	little edian to big edian mode
						send_buf[11] = temp_rx_data[frame_start_position+8];
						send_buf[12] = temp_rx_data[frame_start_position+7];
						send_buf[13] = temp_rx_data[frame_start_position+6];
						send_buf[14] = temp_rx_data[frame_start_position+5];					
						sendToServer(send_buf, 15, LOCK_UP_OP_LOG , 1);
						openLockType = 0;
					}					
					break;
				case LOCK_CMD_ADD_USER:
					if(user_mag.opType == LOCK_CMD_CHANGE_PW){
						Schd_After_Int(500, next_op, 2);								
						break;
					}	else{
						send_buf[0] = LOCK_CMD_ADD_USER;
						if(temp_rx_data[frame_start_position+4] == 0){							
							send_buf[1] = 0;							
						}else{// failed
							send_buf[1] = 1;
						}
						sendToServer(send_buf, 2, UPDATE_CONTENT_CMD , 0);
					}
					break;
				case LOCK_CMD_ADD_PW:
					if(user_mag.opType == LOCK_CMD_CHANGE_PW){
						user_mag.opType = 0;
						send_buf[0] = LOCK_CMD_CHANGE_PW;						
					}else{
						send_buf[0] = LOCK_CMD_ADD_PW;						
					}	
					
					if(temp_rx_data[frame_start_position + 4] == 0){							
						send_buf[1] = 0;							
					}else{// failed
						send_buf[1] = 1;
					}
					sendToServer(send_buf, 2, UPDATE_CONTENT_CMD , 0);					
					break;
				case LOCK_CMD_INIT:
					send_buf[0] = SET_MANUFACTURE_CMD;
					if(temp_rx_data[frame_start_position + 4] == 0){						
						send_buf[1] = 0;						
						enable_doorBell();		
					}else{
						send_buf[1] = 1;
					}		
					sendToServer(send_buf, 2, UPDATE_CONTENT_CMD , 1);					
					break;
				case LOCK_CMD_VOICE_BROADCAST://语音播报
					break;	
				case LOCK_CMD_REGISTER://门锁注册
					if(user_mag.opType == SET_MANUFACTURE_CMD){
						sys_para.sys_fsm = SLEEP_MODE;
					}else{
						send_buf[0] = UPDATE_TIME_REQ;
						sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1);
					}
					break;				
				default: 					
					break;
			}
			if(lock_com.cmd_type != LOCK_CMD_IDLE){
				init_lock_com();
			}
		}
	}
	lock_com.cmd_type = LOCK_CMD_IDLE;
	memset(UART2_RxBuffer, 0, temp_rx_data_len);
	HAL_UART_Receive_IT(&UartHandle2, (uint8_t*)UART2_RxBuffer, UART2_RxBufferSize);
}

//---------    Server    ----------
void serverUpdateNode(uint8_t*data, uint8_t data_len){	
	uint8_t i;
	uint8_t send_buf[23] = {0};
	
	user_mag.opType = data[0];
	switch(user_mag.opType){
		case ADD_FINGER:												
			if( data[data_len - 1] == 0x00 ){			//帧序号 last frame
					up_download_finger.frame_num = data[data_len - 1];
					finger_lib.pageID = data[data_len - 3];
					finger_lib.pageID = ( finger_lib.pageID << 8 ) + data[data_len - 2];
					if(up_download_finger.buf_size != 576)
							up_download_finger.buf_size = 576;
					for( i = 0; i < (data_len - 4); i++ ) {//除去update_content_CMD + frameNum  + flash ID 2bytes
						up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = data[i+1];
					}
					finger_fsm.fsm = REGISTER_DOWN_FSM;	
			}else {
				switch(data[data_len - 1]){
					case 1:	//第一帧数据时，clear buffer: up_download_finger
						up_download_finger.buf_size = 0;
						break;
					case 2:
						if(up_download_finger.buf_size != 192){
							up_download_finger.buf_size = 192;
						}
						break;
					case 3:
						if(up_download_finger.buf_size != 384){
							up_download_finger.buf_size = 384;
						}
						break;
					default:
						break;					
				}			
				for( i = 0; i < (data_len - 2); i++ ) {//除去update_content_CMD + frameNum 
					up_download_finger.up_download_fingerModelBuff[up_download_finger.buf_size++] = data[i+1];
				}
			}
			send_buf[0] = 0xFF;
			sendToServer(send_buf, 1, ACK_CMD, 0 );
			break;
		case DEL_FINGER:	
			if((data[1] == 0xEE) && (data[2] == 0xFF)){
				finger_fsm.fsm = EMPTY_LIB_FSM;
			}else{
				finger_lib.pageID = data[1];
				finger_lib.pageID = (finger_lib.pageID << 8) + data[2];			
				finger_fsm.fsm = DELETE_CHAR_FSM;
			}
			break;
		case LOCK_CMD_ADD_USER:	
			memcpy(send_buf, &data[1], 4);
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_ADD_USER, send_buf, 5);			
			break;
		case LOCK_CMD_ADD_PW:
			memcpy(send_buf, &data[1], 4);
			//send_buf[4-8] = 0;
			send_buf[9] = 0x01; 										//密码类型
			memcpy(&send_buf[10], &data[1], 10);		//密码 = username+ pw
			send_buf[20]= 0;
			send_cmd_to_lock(LOCK_CMD_ADD_PW, send_buf, 21);
			break;
		case LOCK_CMD_DEL_USER:
			memcpy(send_buf, &data[1], 4); 					//user name
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_DEL_USER, send_buf, 5 );		
			if( (data[5]==0xFF) && (data[6] == 0xFF)){ //0xFFFF表示无指纹				
				finger_lib.pageID = 0xFFFF;
			}else if((data[1] == 0xEE) && (data[2] == 0xFF)){
				finger_lib.pageID = 0xEEFF;
				finger_fsm.fsm = EMPTY_LIB_FSM;
				finger_com.cmd_retry_times = 0;
			}else{
				finger_lib.pageID = data[5];
				finger_lib.pageID = (finger_lib.pageID << 8) + data[6];			
				finger_fsm.fsm = DELETE_CHAR_FSM;	
				finger_com.cmd_retry_times = 0;		
			}
			break;
		case LOCK_CMD_DEL_PW:
			memcpy(send_buf, &data[1], 4);
			//send_buf[4-8] = 0;
			send_buf[9] = 0x01; 									//密码类型
			memcpy(&send_buf[10], &data[1], 10);	//密码 = username+ pw
			send_buf[20]= 0;			
			send_cmd_to_lock(LOCK_CMD_DEL_PW, send_buf, 17);
			break;
		case LOCK_CMD_CHANGE_PW:			
			memcpy(user_mag.user_name, &data[1], 4);
			memcpy(user_mag.user_pw, &data[1], 10);
			memcpy(send_buf, user_mag.user_name, 4);
			send_buf[4] = 0;
			send_cmd_to_lock(LOCK_CMD_DEL_USER, send_buf, 5);
			break;	
		case SET_MANUFACTURE_CMD:
			user_mag.opType = SET_MANUFACTURE_CMD;
			finger_fsm.fsm = EMPTY_LIB_FSM;					
			send_cmd_to_lock(LOCK_CMD_INIT, &data[1], 1);						
			break;		
		default:
			break;
	}
}

void process_serverRxBuffer(void){	
//	uint8_t temp_data[2] = {0};
	
	if((lora_data.lora_recv_data_buf[0] == RX_FRAME_HEADER_1) && (lora_data.lora_recv_data_buf[1] == RX_FRAME_HEADER_1)){
		if(lora_data.lora_recv_data_buf[2] == (lora_data.lora_recv_data_buf_len - 2)){
			//sum crc check  ToDo
			if( (lora_data.lora_recv_data_buf[3] == (uint8_t)(lockUnit.nodeID >> 24) ) && (lora_data.lora_recv_data_buf[4] == (uint8_t)(lockUnit.nodeID >> 16))&&\
					(lora_data.lora_recv_data_buf[5] == (uint8_t)(lockUnit.nodeID >> 8)) && (lora_data.lora_recv_data_buf[6] == (uint8_t)lockUnit.nodeID ) ){
					set_ack_flag();					
					switch(lora_data.lora_recv_data_buf[7]){
						case UPDATE_TIME_CMD : // update system time 
							//校准通信板的RTC时钟
							memcpy(sys_para.sys_time, &lora_data.lora_recv_data_buf[8], 6);
							calDateTime(sys_para.sys_time);
							send_cmd_to_lock(LOCK_CMD_CAL_TIME, &lora_data.lora_recv_data_buf[8], 6);	
							update_sysIdle_tick();
							break;
						case UPDATE_CONTENT_CMD:
							//包括子CMD+data+frameNum
							serverUpdateNode(&lora_data.lora_recv_data_buf[8], lora_data.lora_recv_data_buf_len - 10);
							update_sysIdle_tick();
							break;				
						case LOCK_UP_OP_LOG:
							delLastRecord();
							break;
						case SET_SLEEP_CMD:
							if(lora_data.lora_recv_data_buf[8] == 0){		
								if(bell_update == 1){	//闪烁2下			
									powerOn_finger();
									Delay_nms(300);
									powerOff_finger();
									Delay_nms(300);
									powerOn_finger();
									Delay_nms(300);
									powerOff_finger();
//									temp_data[0] = S_SET_SUCCEED;
//									send_cmd_to_lock(LOCK_CMD_VOICE_BROADCAST, temp_data, 1);
								}
								bell_update = 0;
								Schd_After_Int(500, next_op, 6);
//								if(user_mag.opType != SET_MANUFACTURE_CMD){
//									Schd_After_Int(500, next_op, 6);
//								}else{
//									update_sysIdle_tick();
//								}									
							}
							break;
						default:
							break;
					}
			}
		}
	}
	memset(lora_data.lora_recv_data_buf, 0, lora_data.lora_recv_data_buf_len);
	lora_data.lora_recv_data_buf_len = 0;
}
